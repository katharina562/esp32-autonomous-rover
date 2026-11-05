#include <WiFi.h>

#include <WebServer.h>

#include <Wire.h>

#include <VL53L0X.h>

#include <ESP32Servo.h>



// =====================================================

// WIFI

// =====================================================

const char* ssid = "Selina_Rover";

const char* password = "rover123";

WebServer server(80);

// =====================================================

// MOTOR PINS

// =====================================================

const int AIN1 = 25;

const int AIN2 = 26;

const int PWMA = 27;

const int BIN1 = 32;

const int BIN2 = 33;

const int PWMB = 14;

// =====================================================

// SENSOR + SERVO

// =====================================================

const int SDA_PIN = 21;

const int SCL_PIN = 22;

const int PAN_SERVO_PIN = 13;
const int TILT_SERVO_PIN = 23;

VL53L0X sensor;

Servo panServo;
Servo tiltServo;

int currentPanAngle = 90;
int currentTiltAngle = 90;

// =====================================================

// PAN / TILT SETTINGS

// =====================================================

// Limits used by automatic scans only. Manual GUI control remains 0..180°.
int PAN_MIN = 10;
int PAN_MAX = 170;
int TILT_MIN = 50;
int TILT_MAX = 170;

int PAN_CENTER = 90;
int TILT_CENTER = 90;

// 3D scan resolution and settling time.
int SCAN3D_PAN_STEP = 10;
int SCAN3D_TILT_STEP = 10;
int SCAN3D_SERVO_DELAY = 180;

bool scan3DActive = false;

// =====================================================

// AUTONOMOUS SETTINGS

// =====================================================

bool autonomousMode = false;

const int OBSTACLE_DISTANCE = 200;

const int SAFE_DISTANCE = 300;

const int DRIVE_TIME = 400;

const int TURN_TIME = 400;

const int COARSE_STEP = 30;

const int FINE_STEP = 10;

const int REVERSE_TIME = 700;

const int ESCAPE_TURN_TIME = 650;

const int STUCK_LIMIT = 3;

const int STUCK_REVERSE_TIME = 1100;

const int STUCK_TURN_TIME = 900;

int bestScanDistance = 0;

int obstacleCounter = 0;

bool lastEscapeLeft = false;

// =====================================================

// MAPPING SETTINGS

// =====================================================

const int MAP_SCAN_STEP = 5;

const int MAP_MAX_DISTANCE = 2000;

const int MAP_SERVO_DELAY = 180;

bool mappingScanActive = false;

// =====================================================

// WEBSITE

// =====================================================


// =====================================================
// WHEEL ENCODERS / DIFFERENTIAL DRIVE ODOMETRY
// Signed pulse counts are recorded in the interrupt itself.
// One-channel optical encoders infer direction from the motor command.
const int ENCODER_LEFT_PIN = 34;
const int ENCODER_RIGHT_PIN = 35;
const float TICKS_PER_REV = 21.5f;
const float WHEEL_DIAMETER_MM = 65.0f;
// Preliminary linear distance correction: 50 cm measured vs. ~40.4 cm reported.
// Apply the SAME scale to the effective wheelbase, preserving turn angles.
// Recalibrate with a longer straight-line test before treating as final.
const float DISTANCE_SCALE = 1.23f;
const float WHEEL_BASE_MM = 104.0f * DISTANCE_SCALE;
const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM / TICKS_PER_REV) * DISTANCE_SCALE;

// Assumes motor A drives LEFT wheel, motor B drives RIGHT wheel.
// Verify physical mapping before trusting heading sign.
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t pendingSignedLeft = 0, pendingSignedRight = 0;
volatile uint32_t pendingAbsoluteLeft = 0, pendingAbsoluteRight = 0;
volatile int8_t leftWheelDirection = 0, rightWheelDirection = 0;

// Live switch: true = MPU heading, false = original wheel heading.
bool useGyroHeading = true;
float lastGyroUsedDegrees = 0.0f;
extern float gyroTotalDegrees;
extern bool mpuAvailable;
void updateGyro();
float odomXmm = 0, odomYmm = 0, odomTheta = 0;
float odomTotalTheta = 0; // unwrapped angle in radians; for full-turn calibration
float odomDistanceMm = 0; // accumulated absolute center displacement
float odomLeftMm = 0, odomRightMm = 0; // signed wheel distances
uint32_t odomLeftTotal = 0, odomRightTotal = 0;

void IRAM_ATTR onLeftEncoder() {
  portENTER_CRITICAL_ISR(&encoderMux);
  pendingAbsoluteLeft++;
  pendingSignedLeft += leftWheelDirection;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
void IRAM_ATTR onRightEncoder() {
  portENTER_CRITICAL_ISR(&encoderMux);
  pendingAbsoluteRight++;
  pendingSignedRight += rightWheelDirection;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// Diagnostic only: no automatic motor movement until validated on hardware.
// Evaluate actual encoder pulses in rolling 250 ms windows.
// Fault warnings persist until a complete healthy forward move or position reset.
uint32_t faultWindowStartMs = 0;
uint32_t faultLeftPulses = 0, faultRightPulses = 0;
float faultWindowGyroStart = 0;
String wheelFault = "OK";
const uint32_t FAULT_WINDOW_MS = 250;
const uint32_t FAULT_MIN_MOVING_TICKS = 5;

void updateWheelFault(uint32_t absL, uint32_t absR) {
  // Autonomous forward runs are checked per movement in driveForTime().
  if (autonomousMode) return;
  uint32_t now = millis();
  bool bothDriven = (leftWheelDirection != 0 && rightWheelDirection != 0);
  if (!bothDriven) {
    faultWindowStartMs = 0;
    faultLeftPulses = faultRightPulses = 0;
    // Keep last warning until a new valid driving window; useful for inspection.
    return;
  }
  if (faultWindowStartMs == 0) {
    faultWindowStartMs = now;
    faultWindowGyroStart = gyroTotalDegrees;
    faultLeftPulses = faultRightPulses = 0;
  }
  faultLeftPulses += absL;
  faultRightPulses += absR;
  if (now - faultWindowStartMs < FAULT_WINDOW_MS) return;
  uint32_t l = faultLeftPulses, r = faultRightPulses;
  float actualTurn = fabsf(gyroTotalDegrees - faultWindowGyroStart);
  if (l >= FAULT_MIN_MOVING_TICKS && r <= 1) wheelFault = "CHECK RIGHT WHEEL";
  else if (r >= FAULT_MIN_MOVING_TICKS && l <= 1) wheelFault = "CHECK LEFT WHEEL";
  else if (l == 0 && r == 0) wheelFault = "NO ENCODER MOTION";
  else if (l >= FAULT_MIN_MOVING_TICKS && r >= FAULT_MIN_MOVING_TICKS &&
           leftWheelDirection != rightWheelDirection && actualTurn < 2.0f)
    wheelFault = "POSSIBLE WHEEL SLIP";
  // Do not overwrite a previously detected warning with OK.
  faultWindowStartMs = now;
  faultWindowGyroStart = gyroTotalDegrees;
  faultLeftPulses = faultRightPulses = 0;
}

// During autonomous straight legs, encoder distance is integrated at fixed heading.
// This deliberately suppresses both gyro and differential-encoder yaw updates.
extern bool exploreStraightHeadingLock;
void handleExploreMove();
void handleExploreOrient();
void handleExploreState();
void exploreDriveStop(const String &why);
void driveWheels(int left,int right);
void updateOdometry() {
  // Sample the gyro before consuming encoder increments; important when
  // odometry is called by HTTP handlers and not just the main loop.
  updateGyro();
  int32_t signedL, signedR;
  uint32_t absoluteL, absoluteR;
  portENTER_CRITICAL(&encoderMux);
  signedL = pendingSignedLeft;
  signedR = pendingSignedRight;
  absoluteL = pendingAbsoluteLeft;
  absoluteR = pendingAbsoluteRight;
  pendingSignedLeft = pendingSignedRight = 0;
  pendingAbsoluteLeft = pendingAbsoluteRight = 0;
  portEXIT_CRITICAL(&encoderMux);

  updateWheelFault(absoluteL, absoluteR);
  odomLeftTotal += absoluteL;
  odomRightTotal += absoluteR;
  float dL = float(signedL) * MM_PER_TICK;
  float dR = float(signedR) * MM_PER_TICK;
  odomLeftMm += dL;
  odomRightMm += dR;

  float dS = (dL + dR) * 0.5f;
  float encoderDelta = (dR - dL) / WHEEL_BASE_MM;
  float gyroDelta = (gyroTotalDegrees - lastGyroUsedDegrees) * PI / 180.0f;
  lastGyroUsedDegrees = gyroTotalDegrees;
  float dTheta = exploreStraightHeadingLock ? 0.0f : ((useGyroHeading && mpuAvailable) ? gyroDelta : encoderDelta);
  float midTheta = odomTheta + 0.5f * dTheta;
  float travel = (fabsf(dTheta) < 0.00001f)
      ? dS : dS * sinf(0.5f * dTheta) / (0.5f * dTheta);
  odomXmm += travel * cosf(midTheta);
  odomYmm += travel * sinf(midTheta);
  odomTheta += dTheta;
  odomTotalTheta += encoderDelta;
  while (odomTheta > PI) odomTheta -= 2.0f * PI;
  while (odomTheta < -PI) odomTheta += 2.0f * PI;
  odomDistanceMm += fabsf(dS);
}

void setWheelDirections(int left, int right) {
  updateOdometry();
  // Set both direction signs atomically with respect to encoder ISRs.
  portENTER_CRITICAL(&encoderMux);
  leftWheelDirection = (left > 0) ? 1 : ((left < 0) ? -1 : 0);
  rightWheelDirection = (right > 0) ? 1 : ((right < 0) ? -1 : 0);
  portEXIT_CRITICAL(&encoderMux);
}

extern float gyroTotalDegrees;
extern float gyroSegmentDegrees;
extern float gyroRateDps;
extern uint16_t gyroSaturationCount;
extern bool gyroTurning;
extern uint32_t gyroQuietSinceMs;
extern bool mpuAvailable;
void updateGyro();

void handleOdometry() {
  updateGyro();
  updateOdometry();
  String json = "{";
  json += "\"x\":" + String(odomXmm, 1);
  json += ",\"y\":" + String(odomYmm, 1);
  json += ",\"theta\":" + String(odomTheta * 180.0f / PI, 1);
  json += ",\"totalTheta\":" + String(odomTotalTheta * 180.0f / PI, 1);
  json += ",\"gyroAngle\":" + String(gyroTotalDegrees, 1);
  json += ",\"headingMode\":\"" + String((useGyroHeading && mpuAvailable) ? "gyro" : "encoder") + "\"";
  json += ",\"gyroSegment\":" + String(gyroSegmentDegrees, 1);
  json += ",\"gyroRate\":" + String(gyroRateDps, 2);
  json += ",\"gyroSat\":" + String(gyroSaturationCount);
  json += ",\"gyroTurning\":" + String(gyroTurning ? "true" : "false");
  json += ",\"gyroAvailable\":" + String(mpuAvailable ? "true" : "false");
  json += ",\"wheelFault\":\"" + wheelFault + "\"";
  json += ",\"wheelbase\":" + String(WHEEL_BASE_MM, 1);
  json += ",\"distance\":" + String(odomDistanceMm, 1);
  json += ",\"leftTicks\":" + String(odomLeftTotal);
  json += ",\"rightTicks\":" + String(odomRightTotal);
  json += ",\"leftWheelMm\":" + String(odomLeftMm, 1);
  json += ",\"rightWheelMm\":" + String(odomRightMm, 1);
  json += ",\"leftDir\":" + String(int(leftWheelDirection));
  json += ",\"rightDir\":" + String(int(rightWheelDirection));
  json += "}";
  server.send(200, "application/json", json);
}

void handleOdometryMode() {
  updateOdometry(); // finish old-mode motion before switching
  if (!server.hasArg("mode")) {
    server.send(400, "text/plain", "Expected mode=gyro or mode=encoder");
    return;
  }
  String mode = server.arg("mode");
  if (mode != "gyro" && mode != "encoder") {
    server.send(400, "text/plain", "Invalid mode");
    return;
  }
  if (mode == "gyro" && !mpuAvailable) {
    server.send(409, "text/plain", "Gyroscope unavailable");
    return;
  }
  useGyroHeading = (mode == "gyro");
  lastGyroUsedDegrees = gyroTotalDegrees;
  // Switching changes only the future heading increments; position is not reset.
  server.send(200, "text/plain", useGyroHeading ? "GYRO" : "ENCODER");
}

void handleOdometryReset() {
  updateOdometry();
  odomXmm = odomYmm = odomTheta = odomTotalTheta = odomDistanceMm = 0;
  odomLeftMm = odomRightMm = 0;
  odomLeftTotal = odomRightTotal = 0;
  wheelFault = "OK";
  faultWindowStartMs = 0;
  faultLeftPulses = faultRightPulses = 0;
  updateGyro();
  gyroTotalDegrees = 0.0f;
  lastGyroUsedDegrees = 0.0f;
  gyroSegmentDegrees = 0.0f;
  gyroTurning = false;
  gyroQuietSinceMs = 0;
  server.send(200, "text/plain", "ODOMETRY RESET");
}

const char webpage[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"

content="width=device-width, initial-scale=1.0">

<title>Selina Rover</title>

<style>

body {

    font-family: Arial, sans-serif;

    text-align: center;

    background-color: #f3f3f3;

}

h1 {

    margin-top: 30px;

}

h2 {

    margin-top: 35px;

}

#status {

    font-size: 22px;

    margin: 20px;

    font-weight: bold;

}

/* DRIVE */

.controls {

    display: grid;

    grid-template-columns: 90px 90px 90px;

    gap: 10px;

    justify-content: center;

}

.driveButton {

    width: 90px;

    height: 90px;

    font-size: 40px;

    border-radius: 15px;

    border: none;

    background-color: #333;

    color: white;

    cursor: pointer;

}

.stop {

    background-color: #b00020;

}

.empty {

    width: 90px;

    height: 90px;

}

/* SENSOR */

.sensorBox {

    margin: 30px auto;

    padding: 20px;

    max-width: 450px;

    background-color: white;

    border-radius: 15px;

}

#angleDisplay {

    font-size: 28px;

    font-weight: bold;

    margin: 15px;

}

#servoSlider {

    width: 90%;

}

.sensorButton {

    width: 90px;

    height: 50px;

    margin: 5px;

    font-size: 18px;

    border: none;

    border-radius: 10px;

    background-color: #555;

    color: white;

}

/* MAPPING */

.mapBox {

    margin: 30px auto;

    padding: 20px;

    max-width: 520px;

    background-color: white;

    border-radius: 15px;

}

.scanButton {

    width: 220px;

    height: 60px;

    font-size: 18px;

    border: none;

    border-radius: 12px;

    background-color: #1565c0;

    color: white;

    cursor: pointer;

}

#scanStatus {

    margin: 15px;

    font-weight: bold;

}

#mapCanvas {

    width: 100%;

    max-width: 500px;

    border: 1px solid #aaa;

    background-color: #fafafa;

}

/* AUTONOMOUS */

.autoBox {

    margin: 30px auto;

    padding: 25px;

    max-width: 450px;

    background-color: white;

    border-radius: 15px;

}

.autoButton {

    width: 220px;

    height: 65px;

    font-size: 20px;

    border: none;

    border-radius: 12px;

    color: white;

    cursor: pointer;

}

.startAuto {

    background-color: #16803c;

}

.stopAuto {

    background-color: #b00020;

}

#modeDisplay {

    font-size: 22px;

    font-weight: bold;

    margin-bottom: 20px;

}

</style>

</head>

<body>

<h1>Rover Control</h1>

<h2>Manual Drive</h2>

<div id="status">

STOPPED

</div>

<div class="controls">

    <div class="empty"></div>

    <button class="driveButton" id="forward">

        ↑

    </button>

    <div class="empty"></div>

    <button class="driveButton" id="left">

        ←

    </button>

    <button class="driveButton stop" id="stop">

        ■

    </button>

    <button class="driveButton" id="right">

        →

    </button>

    <div class="empty"></div>

    <button class="driveButton" id="backward">

        ↓

    </button>

    <div class="empty"></div>

</div>

<p>Arrow keys or WASD control the rover.</p>

<h2>Wheel Odometry</h2>
<div class="sensorBox">
  <p>Position (mm): <b id="odomXY">0, 0</b></p>
  <p>Heading source: <select id="headingMode"><option value="gyro">Gyroscope</option><option value="encoder">Wheel encoders</option></select> <span id="modeMessage"></span></p>
  <p>Heading: <b id="odomHeading">0°</b> &nbsp; Distance: <b id="odomDistance">0 mm</b></p>
  <p><strong>Calibration: Total turn (unwrapped): <b id="odomTotalTheta">0°</b></strong></p>
  <p><strong>Gyroscope total turn: <b id="gyroAngle">—</b></strong></p>
  <p>Current/last turn segment: <b id="gyroSegment">—</b> &nbsp; Rate: <b id="gyroRate">—</b></p>
  <p>Gyro state: <b id="gyroState">—</b></p>
  <p>Encoder − gyro: <b id="gyroDifference">—</b></p>
  <p style="font-size:13px;">Heading source determines pose updates for both 2D and 3D mapping. Encoder total turn remains a separate comparison. Stop the rover before switching modes; reset pose before comparing maps. Long blocking scans can miss gyro samples.</p>
  <p>Estimated wheelbase after exactly one real 360° turn: <b id="odomCalibratedWheelbase">—</b></p>
  <p style="font-size:13px;">Reset position, turn exactly 360° in one direction, then read the total turn and estimated wheelbase. Repeat left and right several times. Keep DISTANCE_SCALE and WHEEL_BASE_MM consistent when recalibrating. Verify straight distance and full turns.</p>
  <p><strong>Wheel diagnostic: <b id="wheelFault">OK</b></strong></p>
  <p>Encoder ticks: L <b id="odomLeft">0</b> / R <b id="odomRight">0</b></p>
  <p>Signed wheel travel: L <b id="odomLeftMm">0 mm</b> / R <b id="odomRightMm">0 mm</b></p>
  <p id="odomWarning" style="font-size:13px;">Estimated from wheel rotation; slip is not detected.</p>
  <button class="sensorButton" id="odomReset">RESET POSITION</button>
</div>

<!-- SENSOR -->

<h2>Sensor Direction</h2>

<div class="sensorBox">
    <p><b>Pan - Left / Right</b></p>
    <div id="panAngleDisplay">90°</div>
    <input type="range" id="panSlider" min="0" max="180" value="90" step="1">
    <br><br>
    <button class="sensorButton" onclick="setPan(0)">0°</button>
    <button class="sensorButton" onclick="setPan(90)">Center</button>
    <button class="sensorButton" onclick="setPan(180)">180°</button>

    <br><br>
    <p><b>Tilt - Up / Down</b></p>
    <div id="tiltAngleDisplay">90°</div>
    <input type="range" id="tiltSlider" min="0" max="180" value="90" step="1">
    <br><br>
    <button class="sensorButton" onclick="setTilt(0)">0°</button>
    <button class="sensorButton" onclick="setTilt(90)">Center</button>
    <button class="sensorButton" onclick="setTilt(180)">180°</button>
</div>

<!-- MAPPING -->

<h2>Local Environment Map</h2>

<div class="mapBox">

    <button

        id="scanButton"

        class="scanButton">

        SCAN ENVIRONMENT

    </button>

    <div id="scanStatus">

        Ready

    </div>

    <div style="border:1px solid #777;border-radius:8px;padding:12px;margin:12px 0;">
      <h3>Explore Area — Phase 2: map and route preview</h3>
      <p>Distances from starting rover center (cm):</p>
      <label>Forward <input id="exploreFront" type="number" min="0" max="300" value="200" style="width:70px"></label>
      <label>Back <input id="exploreBack" type="number" min="0" max="300" value="0" style="width:70px"></label>
      <label>Left <input id="exploreLeft" type="number" min="0" max="300" value="100" style="width:70px"></label>
      <label>Right <input id="exploreRight" type="number" min="0" max="300" value="100" style="width:70px"></label>
      <p><button id="explorePreview" class="scanButton">SHOW AREA + SAFETY ZONES</button></p>
      <p><button id="exploreFirstScan" class="scanButton">SCAN AREA FROM START (NO DRIVING)</button></p>
      <p><button id="explorePlan" class="scanButton">PREVIEW NEXT TARGET + SAFE PATH</button></p>
      <p><button id="exploreRun" class="scanButton">START MULTI-SCAN EXPLORATION (DRIVE)</button> <button id="exploreStop" class="scanButton">STOP EXPLORATION</button></p>
      <p id="exploreStatus">Test mode: supervised driving only.</p>
      <p style="font-size:12px">Orange = inflated obstacle cells (12 cm center clearance; preview only). Blue = selected area. The rover does not drive automatically yet.</p>
    </div>
    <div style="border:1px solid #1565c0;border-radius:10px;padding:14px;margin:12px 0;">
      <h3>Manual Multi-Scan Mapping (temporary)</h3>
      <p>Drive using WASD / arrow keys. Stop at each location, then add a scan to the same map.</p>
      <button id="manualMapStart" class="scanButton">START NEW MANUAL MAP</button>
      <button id="manualMapScan" class="scanButton" disabled>ADD 2D SCAN</button>
      <button id="manualMapEnd" class="scanButton" disabled>END MANUAL MAPPING</button>
      <p id="manualMapStatus">Inactive</p>
    </div>
    <p>Registered scans: <b id="mapScanCount">0</b></p>
    <button id="resetGlobalMap" class="scanButton">RESET MAP + POSITION</button>
    <div style="display:flex;justify-content:center;align-items:center;gap:10px;flex-wrap:wrap;margin:12px 0">
      <button id="mapZoomOut" type="button">−</button>
      <span id="mapZoomLabel">Map: 3.2 m view</span>
      <button id="mapZoomIn" type="button">+</button>
      <button id="mapZoomFit" type="button">FIT AREA</button>
    </div>
    <canvas

        id="mapCanvas"

        width="500"

        height="500">

    </canvas>

    <p>

        Persistent map in odometry coordinates (browser session)

    </p>

</div>

<!-- 3D SCAN -->

<h2>3D Pan-Tilt Scan</h2>
<div class="mapBox">
    <button id="scan3DButton" class="scanButton">START 3D SCAN</button>
    <div id="scan3DStatus">Ready</div>
    <button id="clear3DMapButton" type="button">CLEAR 3D MAP</button>
    <div id="scan3DCount">0 scans stored</div>
    <canvas id="scan3DCanvas" width="500" height="360" style="width:100%;max-width:500px;border:1px solid #aaa;background:#111;touch-action:none;"></canvas>
    <div style="display:flex;gap:8px;justify-content:center;flex-wrap:wrap;margin-top:8px;">
      <button id="viewLeftButton" type="button">ROTATE LEFT</button>
      <button id="viewResetButton" type="button">RESET VIEW</button>
      <button id="viewRightButton" type="button">ROTATE RIGHT</button>
      <button id="zoomOutButton" type="button">ZOOM -</button>
      <button id="zoomInButton" type="button">ZOOM +</button>
    </div>
    <p>Drag with one finger to rotate. Pinch with two fingers to zoom. The rover remains stationary during the scan.</p>
</div>

<!-- AUTONOMOUS -->

<h2>Autonomous Exploration</h2>

<div class="autoBox">

    <div id="modeDisplay">

        MANUAL MODE

    </div>

    <button

        id="autoButton"

        class="autoButton startAuto">

        START AUTONOMOUS

    </button>

</div>

<script>

// =====================================================

// DRIVE

// =====================================================

function sendCommand(command) {

    fetch("/" + command);

    document.getElementById("status").innerText =

        command.toUpperCase();

}

function setupButton(id, command) {

    const button =

        document.getElementById(id);

    button.addEventListener(

        "mousedown",

        function() {

            sendCommand(command);

        }

    );

    button.addEventListener(

        "mouseup",

        function() {

            sendCommand("stop");

        }

    );

    button.addEventListener(

        "mouseleave",

        function() {

            sendCommand("stop");

        }

    );

    button.addEventListener(

        "touchstart",

        function(event) {

            event.preventDefault();

            sendCommand(command);

        }

    );

    button.addEventListener(

        "touchend",

        function(event) {

            event.preventDefault();

            sendCommand("stop");

        }

    );

}

setupButton("forward", "forward");

setupButton("backward", "backward");

setupButton("left", "left");

setupButton("right", "right");

document

.getElementById("stop")

.addEventListener(

    "click",

    function() {

        sendCommand("stop");

    }

);

// =====================================================

// KEYBOARD

// =====================================================

let currentCommand = "stop";

document.addEventListener(

    "keydown",

    function(event) {

        let command = null;

        if (

            event.key === "ArrowUp" ||

            event.key === "w"

        ) {

            command = "forward";

        }

        else if (

            event.key === "ArrowDown" ||

            event.key === "s"

        ) {

            command = "backward";

        }

        else if (

            event.key === "ArrowLeft" ||

            event.key === "a"

        ) {

            command = "left";

        }

        else if (

            event.key === "ArrowRight" ||

            event.key === "d"

        ) {

            command = "right";

        }

        if (

            command !== null &&

            command !== currentCommand

        ) {

            event.preventDefault();

            currentCommand = command;

            sendCommand(command);

        }

    }

);

document.addEventListener(

    "keyup",

    function(event) {

        if (

            event.key === "ArrowUp" ||

            event.key === "ArrowDown" ||

            event.key === "ArrowLeft" ||

            event.key === "ArrowRight" ||

            event.key === "w" ||

            event.key === "a" ||

            event.key === "s" ||

            event.key === "d"

        ) {

            currentCommand = "stop";

            sendCommand("stop");

        }

    }

);

// =====================================================

// Wheel odometry display; endpoint is read-only except reset.
async function refreshOdometry() {
  try {
    const response = await fetch("/odometry");
    if (!response.ok) return;
    const o = await response.json();
    document.getElementById("headingMode").value = o.headingMode;
    document.getElementById("odomXY").textContent = o.x.toFixed(1) + ", " + o.y.toFixed(1);
    document.getElementById("odomHeading").textContent = o.theta.toFixed(1) + "°";
    document.getElementById("odomTotalTheta").textContent = o.totalTheta.toFixed(1) + "°";
    document.getElementById("gyroAngle").textContent = o.gyroAvailable ? o.gyroAngle.toFixed(1) + "°" : "not available";
    document.getElementById("gyroSegment").textContent = o.gyroAvailable ? o.gyroSegment.toFixed(1) + "°" : "—";
    document.getElementById("gyroRate").textContent = o.gyroAvailable ? o.gyroRate.toFixed(2) + "°/s" : "—";
    document.getElementById("gyroState").textContent = o.gyroAvailable ? (o.gyroTurning ? "ROTATING" : "STILL") : "not available";
    document.getElementById("gyroDifference").textContent = o.gyroAvailable ? (o.totalTheta - o.gyroAngle).toFixed(1) + "°" : "—";
    document.getElementById("odomCalibratedWheelbase").textContent =
      (Math.abs(o.totalTheta) > 1 ? (o.wheelbase * Math.abs(o.totalTheta) / 360).toFixed(1) + " mm" : "—");
    document.getElementById("odomDistance").textContent = o.distance.toFixed(1) + " mm";
    document.getElementById("wheelFault").textContent = o.wheelFault || "—";
    document.getElementById("odomLeft").textContent = o.leftTicks;
    document.getElementById("odomRight").textContent = o.rightTicks;
    document.getElementById("odomLeftMm").textContent = o.leftWheelMm.toFixed(1) + " mm";
    document.getElementById("odomRightMm").textContent = o.rightWheelMm.toFixed(1) + " mm";
  } catch (e) { /* scan may temporarily block HTTP server */ }
}
document.getElementById("odomReset").addEventListener("click", async () => {
  try { await fetch("/odometry/reset"); await refreshOdometry(); } catch (e) {}
});
document.getElementById("headingMode").addEventListener("change", async e => {
  const message = document.getElementById("modeMessage");
  try {
    const response = await fetch("/odometry/mode?mode=" + encodeURIComponent(e.target.value));
    if (!response.ok) throw new Error(await response.text());
    message.textContent = "Mode changed";
    await refreshOdometry();
  } catch (err) {
    message.textContent = err.message;
    await refreshOdometry();
  }
});
setInterval(refreshOdometry, 750);
refreshOdometry();

// PAN / TILT SERVOS
// Pan GUI direction remains reversed, matching the working rover setup.
// Tilt is sent directly. If physical up/down is reversed, change to 180 - guiAngle.
// =====================================================

const panSlider = document.getElementById("panSlider");
const tiltSlider = document.getElementById("tiltSlider");
const panAngleDisplay = document.getElementById("panAngleDisplay");
const tiltAngleDisplay = document.getElementById("tiltAngleDisplay");

panSlider.addEventListener("input", function() {
    const guiAngle = parseInt(panSlider.value);
    panAngleDisplay.innerText = guiAngle + "°";
    const servoAngle = 180 - guiAngle;
    fetch("/pan?angle=" + servoAngle);
});

tiltSlider.addEventListener("input", function() {
    const guiAngle = parseInt(tiltSlider.value);
    tiltAngleDisplay.innerText = guiAngle + "°";
    fetch("/tilt?angle=" + guiAngle);
});

function setPan(angle) {
    panSlider.value = angle;
    panAngleDisplay.innerText = angle + "°";
    fetch("/pan?angle=" + (180 - angle));
}

function setTilt(angle) {
    tiltSlider.value = angle;
    tiltAngleDisplay.innerText = angle + "°";
    fetch("/tilt?angle=" + angle);
}

// =====================================================

// MAPPING

// =====================================================

const mapCanvas =

    document.getElementById("mapCanvas");

const ctx =

    mapCanvas.getContext("2d");

// Visual zoom only: occupancy-grid resolution and sensor measurements are unchanged.
let mapViewSpanMM = 3200;
function updateMapZoomLabel(){
  document.getElementById('mapZoomLabel').textContent = 'Map: ' + (mapViewSpanMM/1000).toFixed(1) + ' m view';
}
document.getElementById('mapZoomIn').addEventListener('click',()=>{
  mapViewSpanMM=Math.max(600,mapViewSpanMM/1.35);updateMapZoomLabel();drawOccupancyGrid();
});
document.getElementById('mapZoomOut').addEventListener('click',()=>{
  mapViewSpanMM=Math.min(8000,mapViewSpanMM*1.35);updateMapZoomLabel();drawOccupancyGrid();
});
document.getElementById('mapZoomFit').addEventListener('click',()=>{
  try {
    const b=getExploreBounds();
    // Fit the selected area with a little room around it.
    mapViewSpanMM=Math.max(600,Math.min(8000,Math.max(b.front+b.back,b.left+b.right)*1.3));
    updateMapZoomLabel();drawOccupancyGrid();
  }catch(e){document.getElementById('exploreStatus').textContent=e.message;}
});
updateMapZoomLabel();

const scanButton =

    document.getElementById("scanButton");

const scanStatus =

    document.getElementById("scanStatus");

// =====================================================

// OCCUPANCY GRID MAP

// =====================================================

// Physical map size

const MAP_WIDTH_MM = 8000;
const MAP_HEIGHT_MM = 8000;
const CELL_SIZE_MM = 50;
const GRID_COLS = MAP_WIDTH_MM / CELL_SIZE_MM;
const GRID_ROWS = MAP_HEIGHT_MM / CELL_SIZE_MM;
// Log-odds occupancy mapping: zero = unknown, negative = free, positive = occupied.
// These values can be tuned after testing.
const LOG_FREE = -0.35;
const LOG_HIT = 0.95;
const LOG_MIN = -3.5;
const LOG_MAX = 3.5;
const ROVER_COL = GRID_COLS / 2;
const ROVER_ROW = GRID_ROWS / 2;
let occupancyGrid = [];
let observationGrid = [];
let currentPose = {x:0, y:0, theta:0};
let lastScanPose = {x:1e6, y:1e6, theta:0};
let scanCount = 0;
let roverTrail = [{x:0,y:0,theta:0}];
let mapBusy = false;
let manualMappingMode = false;
let manualMapFullBounds = false;
let exploreBounds = null;
let explorePlanPath = [];
let explorePlanTarget = null;
const EXPLORE_RADIUS_MM = 140; // minimum center clearance for 20 cm wide rover; NOT zero safety margin
function getExploreBounds() {
  // Manual mapping can use the entire 8 x 8 m grid, independent of the exploration rectangle.
  if (manualMapFullBounds) return {front:4000,back:4000,left:4000,right:4000};
  const ids=['exploreFront','exploreBack','exploreLeft','exploreRight'];
  const values=ids.map(id=>Number(document.getElementById(id).value));
  if(values.some(v=>!Number.isFinite(v)||v<0||v>300)) throw Error('Distances must be between 0 and 300 cm');
  return {front:values[0]*10,back:values[1]*10,left:values[2]*10,right:values[3]*10};
}
function renderExploreOverlay(ctx,cellWidth,cellHeight) {
  if(!exploreBounds)return;
  const b=exploreBounds;
  const px=x=>(ROVER_COL+x/CELL_SIZE_MM+0.5)*cellWidth;
  const py=y=>(ROVER_ROW-y/CELL_SIZE_MM+0.5)*cellHeight;
  ctx.save();
  ctx.strokeStyle='#2478d3';ctx.lineWidth=2;ctx.setLineDash([6,4]);
  ctx.strokeRect(px(-b.left),py(b.front),px(b.right)-px(-b.left),py(-b.back)-py(b.front));
  ctx.setLineDash([]);
  // Inflate every occupied cell by the configured center clearance.
  const rCells=Math.ceil(EXPLORE_RADIUS_MM/CELL_SIZE_MM);
  ctx.fillStyle='rgba(235,135,35,0.24)';
  for(let row=0;row<GRID_ROWS;row++)for(let col=0;col<GRID_COLS;col++){
    if(occupancyGrid[row][col]<0.5)continue;
    for(let dy=-rCells;dy<=rCells;dy++)for(let dx=-rCells;dx<=rCells;dx++){
      if(Math.hypot(dx,dy)*CELL_SIZE_MM>EXPLORE_RADIUS_MM)continue;
      const x=col+dx,y=row+dy;
      if(x>=0&&x<GRID_COLS&&y>=0&&y<GRID_ROWS)
        ctx.fillRect(x*cellWidth,y*cellHeight,cellWidth,cellHeight);
    }
  }
  ctx.restore();
}


// Frontier-based exploration planner (v2). Never issues motor commands itself.
// - Target = reachable known-free cell next to unexplored area inside the boundary,
//   rated by path cost minus information gain (unknown cells around it).
// - Path = Dijkstra over (cell, heading) with 8 neighbours and a TURN PENALTY, then
//   simplified to few straight segments (line of sight). Fewer turns = fewer errors.
// - Every visited target (and every failed one) goes on a blacklist, so unobservable
//   pockets behind obstacles are not approached over and over.
const EXPLORE_TURN_COST_MM_PER_DEG = 2.0;  // a 90 deg turn costs as much as 18 cm of driving
const EXPLORE_GAIN_MM_PER_CELL = 4.0;      // reward per unknown cell around the target
const EXPLORE_MIN_UNKNOWN = 15;            // ignore tiny holes between sensor rays
const EXPLORE_MAX_SEGMENT_MM = 900;        // firmware accepts 5..1200 mm per move
const EXPLORE_MAX_CYCLES = 60;
const EXPLORE_BLOCK_OCC = 0.5;             // one contradicting free ray must not erase a hit (0.95-0.35=0.6)
function exploreLog(msg){
  console.log('EXPLORE: '+msg);
  let el=document.getElementById('exploreLog');
  if(!el){
    el=document.createElement('pre');el.id='exploreLog';
    el.style.cssText='text-align:left;font-size:11px;max-height:160px;overflow:auto;background:#fff;'+
      'border:1px solid #ccc;padding:4px;margin:6px auto;max-width:680px;white-space:pre-wrap';
    const st=document.getElementById('exploreStatus');st.parentNode.insertBefore(el,st.nextSibling);
  }
  el.textContent+=new Date().toLocaleTimeString()+'  '+msg+'\n';
  el.scrollTop=el.scrollHeight;
}
let exploreBlacklist = [];
function exploreSameDirection(a,b,c){
  const ux=b.x-a.x,uy=b.y-a.y,vx=c.x-b.x,vy=c.y-b.y;
  return Math.abs(Math.atan2(ux*vy-uy*vx,ux*vx+uy*vy))<0.05;
}
class ExploreHeap{
  constructor(){this.k=[];this.v=[];}
  get size(){return this.k.length;}
  push(key,val){
    const k=this.k,v=this.v;let i=k.length;k.push(key);v.push(val);
    while(i>0){const p=(i-1)>>1;if(k[p]<=key)break;k[i]=k[p];v[i]=v[p];i=p;}
    k[i]=key;v[i]=val;
  }
  pop(){
    const k=this.k,v=this.v;const topK=k[0],topV=v[0];
    const lastK=k.pop(),lastV=v.pop();const n=k.length;
    if(n>0){
      let i=0;
      while(true){
        let c=2*i+1;if(c>=n)break;
        if(c+1<n&&k[c+1]<k[c])c++;
        if(k[c]>=lastK)break;
        k[i]=k[c];v[i]=v[c];i=c;
      }
      k[i]=lastK;v[i]=lastV;
    }
    return {key:topK,val:topV};
  }
}
function computeExplorePlan() {
  const b=getExploreBounds(); exploreBounds=b;
  const W=GRID_COLS,H=GRID_ROWS,N=W*H;
  const BODY_RADIUS_MM=110;
  const inBounds=(c,r)=>{
    const right=(c-ROVER_COL)*CELL_SIZE_MM;
    const forward=(ROVER_ROW-r)*CELL_SIZE_MM;
    return right>=-b.left+BODY_RADIUS_MM &&
           right<=b.right-BODY_RADIUS_MM &&
           forward>=-b.back &&
           forward<=b.front-BODY_RADIUS_MM;
  };
  const insideArea=(c,r)=>{
    const x=(c-ROVER_COL)*CELL_SIZE_MM,y=(ROVER_ROW-r)*CELL_SIZE_MM;
    return x>=-b.left&&x<=b.right&&y>=-b.back&&y<=b.front;
  };
  // Inflate known obstacles by the rover footprint + margin.
  const blocked=new Uint8Array(N);
  const clearance=EXPLORE_RADIUS_MM;   // centre-to-obstacle clearance (see constant above)
  const radius=Math.ceil(clearance/CELL_SIZE_MM);
  for(let r=0;r<H;r++)for(let c=0;c<W;c++){
    if(!observationGrid[r][c]||occupancyGrid[r][c]<EXPLORE_BLOCK_OCC)continue;
    for(let dr=-radius;dr<=radius;dr++)for(let dc=-radius;dc<=radius;dc++){
      if(Math.hypot(dc,dr)*CELL_SIZE_MM>clearance)continue;
      const rr=r+dr,cc=c+dc;
      if(rr>=0&&rr<H&&cc>=0&&cc<W)blocked[rr*W+cc]=1;
    }
  }
  const start=worldToCell(-currentPose.y,currentPose.x);
  if(start.col<0||start.col>=W||start.row<0||start.row>=H)throw Error('Rover outside map');
  const si=start.row*W+start.col;
  const knownFree=(c,r)=>observationGrid[r][c]&&occupancyGrid[r][c]<0.3;
  // Only KNOWN free cells (plus the rover's own footprint) are driveable.
  const ok=new Uint8Array(N);
  const nearStart=Math.ceil(160/CELL_SIZE_MM);
  for(let r=0;r<H;r++)for(let c=0;c<W;c++){
    const i=r*W+c;
    if(blocked[i]||!inBounds(c,r))continue;
    if(knownFree(c,r)||Math.hypot(c-start.col,r-start.row)<=nearStart)ok[i]=1;
  }
  ok[si]=1;
  // 8 directions, index k = k*45 deg counter-clockwise from "forward".
  const DIRS=[];
  for(let k=0;k<8;k++){
    const phi=k*Math.PI/4;
    DIRS.push({dc:Math.round(-Math.sin(phi))||0,dr:Math.round(-Math.cos(phi))||0,
               len:(k%2?Math.SQRT2:1)*CELL_SIZE_MM});
  }
  const angDiff=(a,b)=>{const d=Math.abs(a-b)%8;return Math.min(d,8-d)*45;};
  const k0=((Math.round(currentPose.theta/45)%8)+8)%8;
  const dist=new Float32Array(N*8).fill(Infinity);
  const prev=new Int32Array(N*8).fill(-1);
  const heap=new ExploreHeap();
  dist[si*8+k0]=0;heap.push(0,si*8+k0);
  while(heap.size){
    const {key,val:s}=heap.pop();
    if(key>dist[s])continue;
    const i=s>>3,d=s&7,c=i%W,r=(i/W)|0;
    for(let nd=0;nd<8;nd++){
      const D=DIRS[nd];
      const cc=c+D.dc,rr=r+D.dr;
      if(cc<0||cc>=W||rr<0||rr>=H)continue;
      const j=rr*W+cc;
      if(!ok[j])continue;
      if(D.dc!==0&&D.dr!==0&&(!ok[r*W+cc]||!ok[rr*W+c]))continue; // no corner cutting
      const cost=key+D.len+EXPLORE_TURN_COST_MM_PER_DEG*angDiff(d,nd);
      const ns=j*8+nd;
      if(cost<dist[ns]){dist[ns]=cost;prev[ns]=s;heap.push(cost,ns);}
    }
  }
  const countUnknown=(c,r,R)=>{
    let n=0;
    for(let dr=-R;dr<=R;dr++)for(let dc=-R;dc<=R;dc++){
      const cc=c+dc,rr=r+dr;
      if(cc<0||cc>=W||rr<0||rr>=H||observationGrid[rr][cc])continue;
      if(insideArea(cc,rr))n++;
    }
    return n;
  };
  let bestState=-1,bestScore=Infinity,reachable=0;
  for(let i=0;i<N;i++){
    if(!ok[i])continue;
    let bd=Infinity,bs=-1;
    for(let d=0;d<8;d++){const v=dist[i*8+d];if(v<bd){bd=v;bs=i*8+d;}}
    if(bs<0)continue;
    reachable++;
    if(bd<100)continue;                         // at least 10 cm away
    const c=i%W,r=(i/W)|0;
    if(!knownFree(c,r))continue;
    const x=(c-ROVER_COL)*CELL_SIZE_MM,y=(ROVER_ROW-r)*CELL_SIZE_MM;
    if(exploreBlacklist.some(p=>Math.hypot(p.x-x,p.y-y)<200))continue;
    if(countUnknown(c,r,2)<1)continue;          // must touch unexplored space
    const g=countUnknown(c,r,5);
    if(g<EXPLORE_MIN_UNKNOWN)continue;          // gaps between rays are not frontiers
    const score=bd-EXPLORE_GAIN_MM_PER_CELL*g;
    if(score<bestScore){bestScore=score;bestState=bs;}
  }
  if(bestState<0)return null;                   // nothing left to explore
  const cells=[];
  for(let s=bestState;s!==-1;s=prev[s])cells.push(s>>3);
  cells.reverse();
  // Line-of-sight simplification: keep only the corners of the route.
  const lineClear=(a,bb)=>{
    let c=a%W,r=(a/W)|0;const c2=bb%W,r2=(bb/W)|0;
    const dc=Math.abs(c2-c),dr=Math.abs(r2-r),sc=c<c2?1:-1,sr=r<r2?1:-1;
    let err=dc-dr;
    while(true){
      if(!ok[r*W+c])return false;
      if(c===c2&&r===r2)return true;
      const e2=2*err;let stepC=false,stepR=false;
      if(e2>-dr){err-=dr;c+=sc;stepC=true;}
      if(e2<dc){err+=dc;r+=sr;stepR=true;}
      if(stepC&&stepR&&(!ok[(r-sr)*W+c]||!ok[r*W+(c-sc)]))return false;
    }
  };
  const pts=[cells[0]];
  let ci=0;
  while(ci<cells.length-1){
    let j=cells.length-1;
    while(j>ci+1&&!lineClear(cells[ci],cells[j]))j--;
    pts.push(cells[j]);ci=j;
  }
  const toMm=i=>({x:((i%W)-ROVER_COL)*CELL_SIZE_MM,y:(ROVER_ROW-((i/W)|0))*CELL_SIZE_MM});
  const wp=pts.map(toMm);
  // Split long segments (firmware limit per move); collinear pieces are driven without extra turns.
  const path=[wp[0]];
  let lengthMm=0;
  for(let k=1;k<wp.length;k++){
    const a=wp[k-1],c2=wp[k];
    const len=Math.hypot(c2.x-a.x,c2.y-a.y);
    lengthMm+=len;
    const n=Math.max(1,Math.ceil(len/EXPLORE_MAX_SEGMENT_MM));
    for(let m=1;m<=n;m++)path.push({x:a.x+(c2.x-a.x)*m/n,y:a.y+(c2.y-a.y)*m/n});
  }
  explorePlanPath=path;
  explorePlanTarget={...path[path.length-1],theta:0};
  return {cells:reachable,steps:path.length-1,target:explorePlanTarget,lengthMm};
}
function renderExplorePlan(ctx,cellWidth,cellHeight){
  if(!explorePlanPath.length)return;
  const px=x=>(ROVER_COL+x/CELL_SIZE_MM+0.5)*cellWidth;
  const py=y=>(ROVER_ROW-y/CELL_SIZE_MM+0.5)*cellHeight;
  ctx.save();ctx.strokeStyle='#19a868';ctx.lineWidth=3;ctx.setLineDash([5,3]);
  ctx.beginPath();explorePlanPath.forEach((p,i)=>i?ctx.lineTo(px(p.x),py(p.y)):ctx.moveTo(px(p.x),py(p.y)));
  ctx.stroke();ctx.setLineDash([]);
  const t=explorePlanTarget;ctx.fillStyle='#12834f';ctx.beginPath();ctx.arc(px(t.x),py(t.y),7,0,2*Math.PI);ctx.fill();ctx.restore();
}

function createEmptyGrid() {
  explorePlanPath=[];explorePlanTarget=null;
  occupancyGrid = Array.from({length:GRID_ROWS}, () => new Float32Array(GRID_COLS));
  observationGrid = Array.from({length:GRID_ROWS}, () => new Uint8Array(GRID_COLS));
}
function updateCell(col,row,increment) {
  if (col < 0 || row < 0 || col >= GRID_COLS || row >= GRID_ROWS) return;
  // Only retain observations inside the user-selected exploration rectangle.
  const b=getExploreBounds();
  const right=(col-ROVER_COL)*CELL_SIZE_MM;
  const forward=(ROVER_ROW-row)*CELL_SIZE_MM;
  if(right < -b.left || right > b.right || forward < -b.back || forward > b.front)return;
  occupancyGrid[row][col] = Math.max(LOG_MIN, Math.min(LOG_MAX,
      occupancyGrid[row][col] + increment));
  observationGrid[row][col] = 1;
}
function worldToCell(x,y) {
  return {
    col: Math.round(ROVER_COL + x/CELL_SIZE_MM),
    row: Math.round(ROVER_ROW - y/CELL_SIZE_MM)
  };
}
function addSensorRay(angleDegrees,distanceMM,pose) {
  if (!Number.isFinite(distanceMM) || distanceMM <= 0 || distanceMM > 2000) return;
  const a = angleDegrees * Math.PI / 180;
  const t = pose.theta * Math.PI / 180;
  // Map coordinates: right and forward. Odom: forward x, left y.
  const roverRight = -pose.y;
  const roverForward = pose.x;
  const rayRight = Math.cos(t)*Math.cos(a) - Math.sin(t)*Math.sin(a);
  const rayForward = Math.sin(t)*Math.cos(a) + Math.cos(t)*Math.sin(a);
  // Sensor is 100 mm ahead of the wheel-axis reference point.
  const sensorRight=roverRight-100*Math.sin(t);
  const sensorForward=roverForward+100*Math.cos(t);
  const start = worldToCell(sensorRight,sensorForward);
  const end = worldToCell(sensorRight+rayRight*distanceMM,
                          sensorForward+rayForward*distanceMM);
  // Bresenham traverses every grid cell once, unlike small step sampling.
  let x=start.col, y=start.row;
  const dx=Math.abs(end.col-x), dy=Math.abs(end.row-y);
  const sx=x<end.col?1:-1, sy=y<end.row?1:-1;
  let err=dx-dy;
  let count=0;
  while ((x!==end.col || y!==end.row) && count++ < GRID_COLS+GRID_ROWS) {
    // Do not mark the sensor's own cell as an obstacle or clear it repeatedly.
    if (count>1) updateCell(x,y,LOG_FREE);
    const e2=2*err;
    if(e2>-dy){err-=dy;x+=sx;}
    if(e2<dx){err+=dx;y+=sy;}
  }
  updateCell(end.col,end.row,LOG_HIT);
}

function drawOccupancyGrid() {

    const width =

        mapCanvas.width;

    const height =

        mapCanvas.height;

    const cellWidth =

        width / GRID_COLS;

    const cellHeight =

        height / GRID_ROWS;

    ctx.clearRect(

        0,

        0,

        width,

        height

    );
    ctx.save();
    const zoom = MAP_WIDTH_MM / mapViewSpanMM;
    const focusX=(ROVER_COL-currentPose.y/CELL_SIZE_MM+0.5)*cellWidth;
    const focusY=(ROVER_ROW-currentPose.x/CELL_SIZE_MM+0.5)*cellHeight;
    ctx.translate(width/2,height/2);
    ctx.scale(zoom,zoom);
    ctx.translate(-focusX,-focusY);


    // -------------------------------------------------

    // DRAW CELLS

    // -------------------------------------------------

    for (

        let row = 0;

        row < GRID_ROWS;

        row++

    ) {

        for (

            let col = 0;

            col < GRID_COLS;

            col++

        ) {

            const bounds=getExploreBounds();
            const right=(col-ROVER_COL)*CELL_SIZE_MM;
            const forward=(ROVER_ROW-row)*CELL_SIZE_MM;
            if(right < -bounds.left || right > bounds.right ||
               forward < -bounds.back || forward > bounds.front)continue;
            const value = occupancyGrid[row][col];
            const seen = observationGrid[row][col];
            if (!seen) {
                ctx.fillStyle = "#bdbdbd";
            } else {
                // Logistic conversion: positive log-odds => dark, negative => light.
                const probability = 1/(1+Math.exp(-value));
                const gray = Math.round(255*(1-probability));
                ctx.fillStyle = `rgb(${gray},${gray},${gray})`;
            }

            ctx.fillRect(

                col * cellWidth,

                row * cellHeight,

                cellWidth + 1,

                cellHeight + 1

            );

        }

    }

    // -------------------------------------------------

    // GRID LINES

    // -------------------------------------------------

    ctx.strokeStyle =

        "rgba(0,0,0,0.08)";

    ctx.lineWidth =

        1;

    for (

        let col = 0;

        col <= GRID_COLS;

        col++

    ) {

        const x =

            col * cellWidth;

        ctx.beginPath();

        ctx.moveTo(

            x,

            0

        );

        ctx.lineTo(

            x,

            height

        );

        ctx.stroke();

    }

    for (

        let row = 0;

        row <= GRID_ROWS;

        row++

    ) {

        const y =

            row * cellHeight;

        ctx.beginPath();

        ctx.moveTo(

            0,

            y

        );

        ctx.lineTo(

            width,

            y

        );

        ctx.stroke();

    }

    // -------------------------------------------------

    renderExploreOverlay(ctx,cellWidth,cellHeight);
    renderExplorePlan(ctx,cellWidth,cellHeight);

    // ROVER POSITION AND TRAIL (world coordinates)
    function canvasPoint(p) {
      return {
        x:(ROVER_COL - p.y/CELL_SIZE_MM + 0.5)*cellWidth,
        y:(ROVER_ROW - p.x/CELL_SIZE_MM + 0.5)*cellHeight
      };
    }
    ctx.strokeStyle = "#e86a33";
    ctx.lineWidth = 2;
    ctx.beginPath();
    roverTrail.forEach((p,i)=> {
      const q=canvasPoint(p);
      if(i===0)ctx.moveTo(q.x,q.y); else ctx.lineTo(q.x,q.y);
    });
    ctx.stroke();
    const rp=canvasPoint(currentPose);
    ctx.fillStyle="red";
    ctx.beginPath(); ctx.arc(rp.x,rp.y,6,0,2*Math.PI); ctx.fill();
    const heading=currentPose.theta*Math.PI/180;
    ctx.strokeStyle="red";ctx.lineWidth=3;
    ctx.beginPath();ctx.moveTo(rp.x,rp.y);
    ctx.lineTo(rp.x-17*Math.sin(heading),rp.y-17*Math.cos(heading));ctx.stroke();
    ctx.restore();
}

function drawScan(result) {
  if (!result || !Array.isArray(result.points) || !result.pose) throw Error("Invalid scan data");
  const data=result.points, pose=result.pose;
  // Each scan is registered using the pose captured when the scan started.
  // Diagnostic: display raw measurements before any map projection.
  const samples = data.filter(p => [0,45,90,135,180].includes(p.angle));
  console.log('Raw ToF scan (mm):', samples.map(p => `${p.angle}°: ${p.distance}`).join(' | '));
  data.forEach(p=>addSensorRay(p.angle,p.distance,pose));
  // No fabricated obstacle endpoints between real sensor measurements.
  // Contradictory scans can now revise cells instead of permanently locking them.
  scanCount++;
  explorePlanPath=[];explorePlanTarget=null;
  currentPose=pose;
  lastScanPose={x:pose.x,y:pose.y,theta:pose.theta};
  roverTrail.push({...pose});
  document.getElementById("mapScanCount").textContent=scanCount;
  drawOccupancyGrid();
}
async function updateMapPose() {
  if(mapBusy)return;
  try {
    const r=await fetch("/odometry");
    if(!r.ok)return;
    const p=await r.json();
    if(!Number.isFinite(p.x)||!Number.isFinite(p.y)||!Number.isFinite(p.theta))return;
    currentPose={x:p.x,y:p.y,theta:p.theta};
    const last=roverTrail[roverTrail.length-1];
    if(Math.hypot(p.x-last.x,p.y-last.y)>40 || Math.abs(p.theta-last.theta)>6) {
      roverTrail.push({...currentPose});
      if(roverTrail.length>2000)roverTrail.shift();
    }
    drawOccupancyGrid();
  }catch(e){}
}
setInterval(updateMapPose,1200);

// Temporary manual mapping workflow. Reuse the existing /mapscan endpoint and drawScan().
const manualMapStart = document.getElementById('manualMapStart');
const manualMapScan = document.getElementById('manualMapScan');
const manualMapEnd = document.getElementById('manualMapEnd');
const manualMapStatus = document.getElementById('manualMapStatus');
manualMapStart.addEventListener('click', async () => {
  if (mapBusy || autonomous || exploreRunning) {
    manualMapStatus.textContent = 'Stop other scanning/exploration first.'; return;
  }
  manualMapStart.disabled = true;
  try {
    await fetch('/stop');
    const response = await fetch('/odometry/reset');
    if (!response.ok) throw Error(await response.text());
    manualMappingMode = true;
    manualMapFullBounds = true;
    exploreBounds = null;
    currentPose = {x:0,y:0,theta:0};
    roverTrail = [{...currentPose}];
    lastScanPose = {x:1e6,y:1e6,theta:0};
    scanCount = 0;
    createEmptyGrid();
    document.getElementById('mapScanCount').textContent = '0';
    manualMapScan.disabled = false;
    manualMapEnd.disabled = false;
    manualMapStatus.textContent = 'Ready. Stop rover and click ADD 2D SCAN.';
    drawOccupancyGrid();
  } catch (e) {
    manualMapStatus.textContent = 'Start failed: ' + e.message;
    manualMapStart.disabled = false;
  }
});
manualMapScan.addEventListener('click', async () => {
  if (!manualMappingMode || mapBusy || autonomous || exploreRunning) return;
  mapBusy = true;
  manualMapScan.disabled = true;
  manualMapStatus.textContent = 'Scanning at current position...';
  try {
    await fetch('/stop');
    const response = await fetch('/mapscan');
    if (!response.ok) throw Error(await response.text());
    drawScan(await response.json());
    manualMapStatus.textContent = scanCount + ' scans registered. Drive to next position.';
  } catch (e) { manualMapStatus.textContent = 'Scan failed: ' + e.message; }
  finally { mapBusy = false; manualMapScan.disabled = !manualMappingMode; }
});
manualMapEnd.addEventListener('click', async () => {
  if (mapBusy) return;
  await fetch('/stop');
  manualMappingMode = false;
  manualMapScan.disabled = true;
  manualMapEnd.disabled = true;
  manualMapStart.disabled = false;
  manualMapStatus.textContent = 'Finished. Existing scans remain visible until reset/new map.';
  drawOccupancyGrid();
});

document.getElementById("resetGlobalMap").addEventListener("click",async()=>{
  if(mapBusy || autonomous)return;
  try {
    const r=await fetch("/odometry/reset");
    if(!r.ok)throw Error("Reset failed");
    currentPose={x:0,y:0,theta:0};roverTrail=[{...currentPose}];
    manualMappingMode=false;manualMapFullBounds=false;
    manualMapScan.disabled=true;manualMapEnd.disabled=true;manualMapStart.disabled=false;
    scanCount=0;createEmptyGrid();drawOccupancyGrid();
    document.getElementById("mapScanCount").textContent="0";
    scanStatus.textContent="Map and odometry reset";
  }catch(e){scanStatus.textContent="Reset failed";}
});

document.getElementById('explorePreview').addEventListener('click',()=>{
  try {
    exploreBounds=getExploreBounds();
    document.getElementById('exploreStatus').textContent='Area displayed. Orange safety zones are visualization only; no autonomous driving.';
    drawOccupancyGrid();
  } catch(e){document.getElementById('exploreStatus').textContent=e.message;}
});
document.getElementById('explorePlan').addEventListener('click',()=>{
  const status=document.getElementById('exploreStatus');
  if(mapBusy||autonomous){status.textContent='Stop scanning/autonomous mode first.';return;}
  try{
    explorePlanPath=[];explorePlanTarget=null;
    const plan=computeExplorePlan();drawOccupancyGrid();
    if(!plan){status.textContent='No reachable frontier left: exploration would be finished.';return;}
    status.textContent='GREEN dashed line = proposed route ('+(plan.lengthMm/10).toFixed(0)+' cm), dot = next scan position. PREVIEW ONLY — no driving.';
  }catch(e){explorePlanPath=[];explorePlanTarget=null;drawOccupancyGrid();status.textContent='No safe path: '+e.message;}
});
document.getElementById('exploreFirstScan').addEventListener('click',async()=>{
  const status=document.getElementById('exploreStatus');
  if(mapBusy||autonomous){status.textContent='Stop current scan/autonomous mode first.';return;}
  try{
    exploreBounds=getExploreBounds();
    drawOccupancyGrid();
    mapBusy=true;
    scanButton.disabled=true;
    document.getElementById('exploreFirstScan').disabled=true;
    status.textContent='Scanning from current position; rover will not move.';
    const response=await fetch('/mapscan');
    if(!response.ok)throw Error('Scan request failed: '+response.status);
    drawScan(await response.json());
    status.textContent='First scan complete. Check blue boundary and orange inflated obstacles.';
  }catch(e){status.textContent='Scan failed: '+e.message;}
  finally{mapBusy=false;scanButton.disabled=false;document.getElementById('exploreFirstScan').disabled=false;}
});

let exploreRunning=false;
let exploreObstacleRecovery=false;
const exploreWait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function exploreStopDrive(){
  exploreRunning=false;
  try{await fetch('/explore/stop');}catch(e){}
}
document.getElementById('exploreStop').addEventListener('click',exploreStopDrive);
async function exploreMoveCell(p,straightOnly=false,finalLeg=false){
  // Map coordinates: x=right, y=forward. Odometry: x=forward, y=left.
  const wx=p.y,wy=-p.x;
  const response=await fetch('/explore/move?x='+encodeURIComponent(wx)+'&y='+encodeURIComponent(wy)+
    (straightOnly?'&straight=1':'')+(finalLeg?'&final=1':''));
  if(!response.ok)throw Error('Move rejected: '+await response.text());
  const started=Date.now();
  while(exploreRunning){
    await exploreWait(120);
    const state=await (await fetch('/explore/state')).json();
    if(!state.active){
      if(state.range>0){
        // The forward sensor stopped the rover: enter that hit into the map so the
        // planner knows the obstacle and does not drive into it again.
        addSensorRay(90,state.range,{x:state.ox,y:state.oy,theta:state.oth});
        drawOccupancyGrid();
        exploreLog('  sensor stop: obstacle '+state.range+' mm ahead');
      }
      if(state.status==='avoided'){exploreObstacleRecovery=true;return;}
      if(state.status!=='arrived')throw Error('Drive stopped: '+state.status+' ('+state.rem+' mm to waypoint)');
      return;
    }
    if(Date.now()-started>35000)throw Error('Waypoint watchdog timeout');
  }
  throw Error('Stopped by user');
}
document.getElementById('exploreRun').addEventListener('click',async()=>{
  const status=document.getElementById('exploreStatus');
  if(exploreRunning||mapBusy||autonomous)return;
  exploreRunning=true;
  document.getElementById('exploreRun').disabled=true;
  let consecutiveFailures=0, finished=false;
  const failCount=new Map();
  try{
    exploreBounds=getExploreBounds();
    exploreBlacklist=[];
    exploreLog('--- exploration started ---');
    for(let cycle=0;cycle<EXPLORE_MAX_CYCLES&&exploreRunning;cycle++){
      if(scanCount===0){
        mapBusy=true;
        const r=await fetch('/mapscan');if(!r.ok)throw Error('Initial scan failed');
        drawScan(await r.json());mapBusy=false;
      }
      await updateMapPose();
      const plan=computeExplorePlan();drawOccupancyGrid();
      if(!plan){finished=true;break;}          // no reachable frontier left = map is complete
      const path=explorePlanPath.slice();
      const target={x:plan.target.x,y:plan.target.y};
      const key=Math.round(target.x)+','+Math.round(target.y);
      exploreLog('cycle '+(cycle+1)+': target ('+Math.round(target.x)+','+Math.round(target.y)+') mm, '+
                 (plan.lengthMm/10).toFixed(0)+' cm, '+(path.length-1)+' legs');
      status.textContent='Cycle '+(cycle+1)+': driving '+(plan.lengthMm/10).toFixed(0)+' cm to the next frontier';
      let outcome='arrived',reason='';
      try{
        for(let k=1;k<path.length&&exploreRunning;k++){
          // Collinear continuation of the same line: the firmware skips the turn.
          const straight=k>1&&exploreSameDirection(path[k-2],path[k-1],path[k]);
          await exploreMoveCell(path[k],straight,k===path.length-1);
          if(exploreObstacleRecovery){outcome='obstacle';reason='obstacle avoided (reversed and turned)';break;}
        }
      }catch(e){
        if(!String(e.message).startsWith('Drive stopped'))throw e;
        outcome='failed';reason=e.message;
      }
      if(!exploreRunning)break;
      exploreObstacleRecovery=false;
      await updateMapPose();                    // where did the rover really end up?
      const dTarget=Math.hypot(target.x-(-currentPose.y),target.y-currentPose.x);
      if(outcome!=='arrived'){
        const n=(failCount.get(key)||0)+1;failCount.set(key,n);
        exploreLog('  '+outcome+': '+reason+' | '+Math.round(dTarget)+' mm from target | try '+n);
        if(dTarget<250){outcome='arrived';exploreLog('  close enough: counts as arrived');}
        else{
          consecutiveFailures++;
          if(n>=2){exploreBlacklist.push(target);exploreLog('  target given up');}
          if(consecutiveFailures>=6)throw Error('6 failed drives in a row, last: '+reason);
        }
      }
      if(outcome==='arrived'){consecutiveFailures=0;exploreBlacklist.push(target);}
      // Scan only if the pose really changed; the same view again adds nothing.
      const moved=Math.hypot(currentPose.x-lastScanPose.x,currentPose.y-lastScanPose.y);
      const turned=Math.abs(((currentPose.theta-lastScanPose.theta+540)%360)-180);
      if(moved>80||turned>25){
        mapBusy=true;
        status.textContent='Cycle '+(cycle+1)+': scanning';
        const r=await fetch('/mapscan');if(!r.ok)throw Error('Scan failed: '+r.status);
        drawScan(await r.json());mapBusy=false;
      }else exploreLog('  pose unchanged since the last scan: planning again without scanning');
    }
    if(finished){status.textContent='Exploration finished: no reachable frontier left.';exploreLog('--- finished ---');}
    else if(exploreRunning)status.textContent='Stopped after maximum '+EXPLORE_MAX_CYCLES+' scan cycles.';
  }catch(e){status.textContent='Exploration halted: '+e.message;exploreLog('HALTED: '+e.message);}
  finally{await exploreStopDrive();mapBusy=false;document.getElementById('exploreRun').disabled=false;}
});

createEmptyGrid();

drawOccupancyGrid();

scanButton.addEventListener(

    "click",

    function() {

        if (autonomous) {

            scanStatus.innerText =

                "Stop autonomous mode first.";

            return;

        }

        mapBusy = true;
        scanButton.disabled =

            true;

        scanStatus.innerText =

            "Scanning...";

        fetch("/mapscan")

        .then(

            response => {

                if (!response.ok) {

                    throw new Error(

                        "Scan failed"

                    );

                }

                return response.json();

            }

        )

        .then(

            data => {

                drawScan(data);

                scanStatus.innerText =

                    "Scan complete";

                mapBusy = false;
                scanButton.disabled =

                    false;

            }

        )

        .catch(

            error => {

                scanStatus.innerText =

                    "Scan error";

                mapBusy = false;
                scanButton.disabled =

                    false;

                console.log(error);

            }

        );

    }

);

// =====================================================
// 3D SCAN
// =====================================================

const scan3DButton = document.getElementById("scan3DButton");
const scan3DStatus = document.getElementById("scan3DStatus");
const scan3DCanvas = document.getElementById("scan3DCanvas");
const ctx3D = scan3DCanvas.getContext("2d");

let scan3DPoints = [];
let scan3DScans = [];
const scan3DColors = ["#6ee7ff", "#fbbf24", "#f472b6", "#a3e635", "#c4b5fd", "#fb923c"];
let viewYaw = -0.45;
let viewPitch = -0.35;
let viewZoom = 0.13;
const active3DPointers = new Map();
let lastPointerX = 0;
let lastPointerY = 0;
let lastPinchDistance = 0;

function convert3DScan(data, pose) {
    // Odometry uses X=forward and Y=left, while the 3D viewer uses
    // X=right and Y=forward. Transform each scan into the initial
    // rover frame, keeping previously acquired scans unchanged.
    const heading = pose.theta * Math.PI / 180;
    const c = Math.cos(heading), si = Math.sin(heading);
    const originX = -pose.y;
    const originY = pose.x;
    const color = scan3DColors[scan3DScans.length % scan3DColors.length];
    const points = [];
    data.forEach(p => {
        if (p.distance <= 0 || p.distance > 2000) return;
        const az = (p.pan - 90) * Math.PI / 180;
        const el = (p.tilt - 90) * Math.PI / 180;
        const horizontal = p.distance * Math.cos(el);
        const localX = -horizontal * Math.sin(az);
        const localY = horizontal * Math.cos(az);
        points.push({
            x: originX + localX * c - localY * si,
            y: originY + localX * si + localY * c,
            z: p.distance * Math.sin(el),
            distance: p.distance,
            color: color
        });
    });
    scan3DScans.push({points: points, x: originX, y: originY, theta: heading, color: color});
    scan3DPoints = scan3DScans.flatMap(scan => scan.points);
    document.getElementById("scan3DCount").textContent =
        scan3DScans.length + " scans stored / " + scan3DPoints.length + " points";
}

function rotate3DPoint(p) {
    // Yaw around vertical Z axis.
    const cy = Math.cos(viewYaw), sy = Math.sin(viewYaw);
    const x1 = p.x * cy - p.y * sy;
    const y1 = p.x * sy + p.y * cy;
    const z1 = p.z;

    // Pitch around horizontal X axis.
    const cp = Math.cos(viewPitch), sp = Math.sin(viewPitch);
    return {
        x: x1,
        y: y1 * cp - z1 * sp,
        z: y1 * sp + z1 * cp
    };
}

function project3D(p, w, h) {
    const r = rotate3DPoint(p);
    // Mild perspective: farther points become slightly smaller.
    const perspective = 900 / Math.max(350, 900 + r.y * 0.22);
    return {
        x: w / 2 + r.x * viewZoom * perspective,
        y: h / 2 - r.z * viewZoom * perspective,
        depth: r.y,
        scale: perspective
    };
}

function draw3DAxis(x, y, z, label, color) {
    const w = scan3DCanvas.width, h = scan3DCanvas.height;
    const a = project3D({x:0,y:0,z:0}, w, h);
    const b = project3D({x:x,y:y,z:z}, w, h);
    ctx3D.strokeStyle = color;
    ctx3D.lineWidth = 2;
    ctx3D.beginPath(); ctx3D.moveTo(a.x,a.y); ctx3D.lineTo(b.x,b.y); ctx3D.stroke();
    ctx3D.fillStyle = color;
    ctx3D.font = "12px Arial";
    ctx3D.fillText(label, b.x + 4, b.y - 4);
}

function draw3DPointCloud() {
    const w = scan3DCanvas.width;
    const h = scan3DCanvas.height;
    ctx3D.clearRect(0, 0, w, h);
    ctx3D.fillStyle = "#111";
    ctx3D.fillRect(0, 0, w, h);

    draw3DAxis(350,0,0,"X", "#ff7777");
    draw3DAxis(0,350,0,"Y", "#77ff99");
    draw3DAxis(0,0,350,"Z", "#77aaff");

    const projected = scan3DPoints.map(p => {
        const q = project3D(p, w, h);
        q.distance = p.distance;
        q.color = p.color;
        return q;
    });
    projected.sort((a,b) => b.depth - a.depth);

    projected.forEach(p => {
        const radius = Math.max(1.5, Math.min(4, 2.4 * p.scale));
        ctx3D.fillStyle = p.color;
        ctx3D.beginPath();
        ctx3D.arc(p.x, p.y, radius, 0, 2 * Math.PI);
        ctx3D.fill();
    });

    scan3DScans.forEach((scan, index) => {
        const rover = project3D({x:scan.x,y:scan.y,z:0}, w, h);
        ctx3D.fillStyle = scan.color;
        ctx3D.beginPath();
        ctx3D.arc(rover.x, rover.y, 5, 0, 2 * Math.PI);
        ctx3D.fill();
        ctx3D.font = "11px Arial";
        ctx3D.fillText("S" + (index + 1), rover.x + 7, rover.y - 5);
    });
}

function reset3DView() {
    viewYaw = -0.45;
    viewPitch = -0.35;
    viewZoom = 0.13;
    draw3DPointCloud();
}

scan3DCanvas.addEventListener("pointerdown", e => {
    scan3DCanvas.setPointerCapture(e.pointerId);
    active3DPointers.set(e.pointerId, {x:e.clientX, y:e.clientY});
    if (active3DPointers.size === 1) {
        lastPointerX = e.clientX; lastPointerY = e.clientY;
    } else if (active3DPointers.size === 2) {
        const pts = Array.from(active3DPointers.values());
        lastPinchDistance = Math.hypot(pts[0].x-pts[1].x, pts[0].y-pts[1].y);
    }
});

scan3DCanvas.addEventListener("pointermove", e => {
    if (!active3DPointers.has(e.pointerId)) return;
    active3DPointers.set(e.pointerId, {x:e.clientX, y:e.clientY});
    if (active3DPointers.size === 1) {
        const dx = e.clientX - lastPointerX;
        const dy = e.clientY - lastPointerY;
        viewYaw += dx * 0.012;
        viewPitch += dy * 0.012;
        viewPitch = Math.max(-1.5, Math.min(1.5, viewPitch));
        lastPointerX = e.clientX; lastPointerY = e.clientY;
        draw3DPointCloud();
    } else if (active3DPointers.size === 2) {
        const pts = Array.from(active3DPointers.values());
        const d = Math.hypot(pts[0].x-pts[1].x, pts[0].y-pts[1].y);
        if (lastPinchDistance > 0) {
            viewZoom *= d / lastPinchDistance;
            viewZoom = Math.max(0.035, Math.min(0.65, viewZoom));
        }
        lastPinchDistance = d;
        draw3DPointCloud();
    }
});

function end3DPointer(e) {
    active3DPointers.delete(e.pointerId);
    if (active3DPointers.size < 2) lastPinchDistance = 0;
    if (active3DPointers.size === 1) {
        const p = Array.from(active3DPointers.values())[0];
        lastPointerX = p.x; lastPointerY = p.y;
    }
}
scan3DCanvas.addEventListener("pointerup", end3DPointer);
scan3DCanvas.addEventListener("pointercancel", end3DPointer);

// Mouse-wheel zoom for laptop/desktop use.
scan3DCanvas.addEventListener("wheel", e => {
    e.preventDefault();
    viewZoom *= e.deltaY < 0 ? 1.12 : 0.89;
    viewZoom = Math.max(0.035, Math.min(0.65, viewZoom));
    draw3DPointCloud();
}, {passive:false});

document.getElementById("viewLeftButton").addEventListener("click", () => { viewYaw -= 0.25; draw3DPointCloud(); });
document.getElementById("viewRightButton").addEventListener("click", () => { viewYaw += 0.25; draw3DPointCloud(); });
document.getElementById("zoomInButton").addEventListener("click", () => { viewZoom=Math.min(0.65,viewZoom*1.2); draw3DPointCloud(); });
document.getElementById("zoomOutButton").addEventListener("click", () => { viewZoom=Math.max(0.035,viewZoom/1.2); draw3DPointCloud(); });
document.getElementById("viewResetButton").addEventListener("click", reset3DView);

document.getElementById("clear3DMapButton").addEventListener("click", () => {
    scan3DScans = [];
    scan3DPoints = [];
    document.getElementById("scan3DCount").textContent = "0 scans stored";
    scan3DStatus.innerText = "3D map cleared. Reset odometry before starting a new map.";
    draw3DPointCloud();
});

scan3DButton.addEventListener("click", async function() {
    if (autonomous) {
        scan3DStatus.innerText = "Stop autonomous mode first.";
        return;
    }
    scan3DButton.disabled = true;
    scan3DStatus.innerText = "Reading pose and scanning...";
    try {
        // Capture pose BEFORE scanning. The rover is stopped throughout the scan.
        const poseResponse = await fetch("/odometry");
        if (!poseResponse.ok) throw new Error("Could not read odometry");
        const pose = await poseResponse.json();
        const response = await fetch("/scan3d");
        if (!response.ok) throw new Error("3D scan failed");
        const data = await response.json();
        convert3DScan(data, pose);
        draw3DPointCloud();
        scan3DStatus.innerText = "Scan " + scan3DScans.length + " complete: " + data.length + " measurements";
    } catch (error) {
        scan3DStatus.innerText = "3D scan error: " + error.message;
        console.log(error);
    } finally {
        scan3DButton.disabled = false;
    }
});

// =====================================================

// AUTONOMOUS

// =====================================================

let autonomous = false;

const autoButton =

    document.getElementById("autoButton");

const modeDisplay =

    document.getElementById("modeDisplay");

autoButton.addEventListener(

    "click",

    function() {

        if (!autonomous) {

            fetch("/auto/start");

            autonomous = true;

            modeDisplay.innerText =

                "AUTONOMOUS MODE";

            autoButton.innerText =

                "STOP AUTONOMOUS";

            autoButton.classList.remove(

                "startAuto"

            );

            autoButton.classList.add(

                "stopAuto"

            );

        }

        else {

            fetch("/auto/stop");

            autonomous = false;

            modeDisplay.innerText =

                "MANUAL MODE";

            autoButton.innerText =

                "START AUTONOMOUS";

            autoButton.classList.remove(

                "stopAuto"

            );

            autoButton.classList.add(

                "startAuto"

            );

        }

    }

);

</script>

</body>

</html>

)rawliteral";

// =====================================================

// MOTOR CONTROL

//

// IMPORTANT:

// NO analogWrite()

// NO LEDC

// NO speed correction

//

// PWMA/PWMB are only HIGH or LOW.

// =====================================================

void forward() {
  setWheelDirections(1, 1);


  digitalWrite(AIN1, HIGH);

  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);

  digitalWrite(BIN2, LOW);

  digitalWrite(PWMA, HIGH);

  digitalWrite(PWMB, HIGH);

}

void backward() {
  setWheelDirections(-1, -1);


  digitalWrite(AIN1, LOW);

  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);

  digitalWrite(BIN2, HIGH);

  digitalWrite(PWMA, HIGH);

  digitalWrite(PWMB, HIGH);

}

void turnLeft() {
  setWheelDirections(-1, 1);


  digitalWrite(AIN1, LOW);

  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);

  digitalWrite(BIN2, LOW);

  digitalWrite(PWMA, HIGH);

  digitalWrite(PWMB, HIGH);

}

void turnRight() {
  setWheelDirections(1, -1);


  digitalWrite(AIN1, HIGH);

  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);

  digitalWrite(BIN2, HIGH);

  digitalWrite(PWMA, HIGH);

  digitalWrite(PWMB, HIGH);

}

void stopMotors() {
  setWheelDirections(0, 0);


  digitalWrite(PWMA, LOW);

  digitalWrite(PWMB, LOW);

  digitalWrite(AIN1, LOW);

  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);

  digitalWrite(BIN2, LOW);

}

// =====================================================

// =====================================================
// BOUNDED AUTONOMOUS WHEEL RECOVERY
// =====================================================
// Short reverse/forward pulses; never run indefinitely.
const uint8_t RECOVERY_MAX_ATTEMPTS = 8;
const uint32_t RECOVERY_PULSE_MS = 170;
const uint32_t RECOVERY_PAUSE_MS = 90;
const uint32_t RECOVERY_MIN_TICKS = 2;

// Keep servicing the web server so /auto/stop works during recovery.
bool recoveryWait(uint32_t durationMs) {
  uint32_t started = millis();
  while (millis() - started < durationMs) {
    server.handleClient();
    updateOdometry();
    if (!autonomousMode) { stopMotors(); return false; }
    delay(5);
  }
  return true;
}

bool recoverStuckWheel(const String &detectedFault) {
  // Unexpected turns alone are not enough evidence for a wheel to be stuck.
  const bool checkLeft = detectedFault == "CHECK LEFT WHEEL" || detectedFault == "NO ENCODER MOTION";
  const bool checkRight = detectedFault == "CHECK RIGHT WHEEL" || detectedFault == "NO ENCODER MOTION";
  if (!checkLeft && !checkRight) return true;

  stopMotors();
  wheelFault = "RECOVERING: " + detectedFault;
  if (!recoveryWait(RECOVERY_PAUSE_MS)) return false;

  for (uint8_t attempt = 1; attempt <= RECOVERY_MAX_ATTEMPTS; ++attempt) {
    if (!autonomousMode) { stopMotors(); return false; }
    updateOdometry();
    const uint32_t l0 = odomLeftTotal;
    const uint32_t r0 = odomRightTotal;
    wheelFault = "RECOVERING " + String(attempt) + "/" + String(RECOVERY_MAX_ATTEMPTS);

    backward();
    if (!recoveryWait(RECOVERY_PULSE_MS)) return false;
    stopMotors();
    if (!recoveryWait(RECOVERY_PAUSE_MS)) return false;

    forward();
    if (!recoveryWait(RECOVERY_PULSE_MS)) return false;
    stopMotors();
    updateOdometry();
    const uint32_t dl = odomLeftTotal - l0;
    const uint32_t dr = odomRightTotal - r0;

    Serial.printf("Recovery %u: L=%lu R=%lu ticks\n", attempt,
                  (unsigned long)dl, (unsigned long)dr);
    // Require evidence from the wheel(s) that were previously stuck.
    if ((!checkLeft || dl >= RECOVERY_MIN_TICKS) &&
        (!checkRight || dr >= RECOVERY_MIN_TICKS)) {
      wheelFault = "OK";
      return true;
    }
    if (!recoveryWait(RECOVERY_PAUSE_MS)) return false;
  }

  stopMotors();
  autonomousMode = false; // Fail safe: don't continue driving into an obstruction.
  wheelFault = "RECOVERY FAILED - AUTO STOPPED";
  return false;
}

// TIMED MOVEMENT

// =====================================================

bool driveForTime(

    void (*movement)(),

    int duration

) {

  // Evaluate each autonomous forward command independently, rather than
  // waiting for a diagnostic window that can span multiple short moves.
  const bool diagnoseForward = autonomousMode && (movement == forward);
  updateOdometry();  // consume pulses left over from the preceding action
  const uint32_t startLeftTicks = odomLeftTotal;
  const uint32_t startRightTicks = odomRightTotal;
  const float startGyroDegrees = gyroTotalDegrees;
  uint32_t lastDiagnosticMs = millis();
  bool faultDetectedThisMove = false;

  movement();

  unsigned long start =

      millis();

  while (

      millis() - start <

      (unsigned long)duration

  ) {

    server.handleClient();
    updateOdometry();
    updateGyro();

    if (diagnoseForward && millis() - lastDiagnosticMs >= 50) {
      lastDiagnosticMs = millis();
      const uint32_t l = odomLeftTotal - startLeftTicks;
      const uint32_t r = odomRightTotal - startRightTicks;
      const float turn = fabsf(gyroTotalDegrees - startGyroDegrees);
      const uint32_t elapsed = millis() - start;
      // A stopped wheel is suspected if its partner has already moved.
      if (l >= 3 && r <= 1) {
        wheelFault = "CHECK RIGHT WHEEL";
        faultDetectedThisMove = true;
      } else if (r >= 3 && l <= 1) {
        wheelFault = "CHECK LEFT WHEEL";
        faultDetectedThisMove = true;
      } else if (elapsed >= 300 && l == 0 && r == 0) {
        wheelFault = "NO ENCODER MOTION";
        faultDetectedThisMove = true;
      } else if (elapsed >= 100 && turn >= 8.0f &&
                 abs((int32_t)l - (int32_t)r) >= 3) {
        wheelFault = "UNEXPECTED TURN / POSSIBLE SLIP";
        faultDetectedThisMove = true;
      } else if (elapsed >= 100 && turn >= 8.0f) {
        wheelFault = "UNEXPECTED TURN";
        faultDetectedThisMove = true;
      }
      // Do not clear a fault until an entire subsequent move is healthy.
      if (faultDetectedThisMove) {
        stopMotors();
        break; // React within this movement, not after its full 400 ms.
      }
    }

    if (!autonomousMode) {

      stopMotors();

        tiltServo.write(TILT_CENTER);
        currentTiltAngle = TILT_CENTER;

      return false;

    }

    delay(10);

  }

  stopMotors();

  if (diagnoseForward && faultDetectedThisMove) {
    const String detectedFault = wheelFault;
    if (!recoverStuckWheel(detectedFault)) return false;
    // Recovery finished; skip the interrupted navigation move. A fresh
    // autonomousStep() will scan before choosing the next direction.
    return true;
  }

  if (diagnoseForward && !faultDetectedThisMove) {
    updateOdometry();
    const uint32_t l = odomLeftTotal - startLeftTicks;
    const uint32_t r = odomRightTotal - startRightTicks;
    const float turn = fabsf(gyroTotalDegrees - startGyroDegrees);
    // Only clear an old warning after a complete, plausible forward move.
    // Too few ticks are inconclusive, not proof that the wheel recovered.
    if (l >= 3 && r >= 3 &&
        abs((int32_t)l - (int32_t)r) <= 2 && turn < 8.0f) {
      wheelFault = "OK";
    }
  }

  return true;

}

// =====================================================

// DISTANCE SENSOR

// =====================================================

int readDistance() {

  uint16_t distance =

      sensor.readRangeContinuousMillimeters();

  if (

      sensor.timeoutOccurred()

  ) {

    Serial.println(

        "SENSOR TIMEOUT"

    );

    return 0;

  }

  return distance;

}

// =====================================================

// COARSE SCAN

// =====================================================

bool coarseScan() {

  bool obstacleAhead =

      false;

  Serial.println();

  Serial.println(

      "COARSE SCAN"

  );

  // Navigation uses a fast horizontal scan.
  tiltServo.write(TILT_CENTER);
  currentTiltAngle = TILT_CENTER;
  delay(200);

  for (

      int angle = PAN_MIN;

      angle <= PAN_MAX;

      angle += COARSE_STEP

  ) {

    server.handleClient();

    if (!autonomousMode) {

      return true;

    }

    panServo.write(

        angle

    );

    currentPanAngle =

        angle;

    delay(350);

    int distance =

        readDistance();

    Serial.print(angle);

    Serial.print(" deg -> ");

    Serial.print(distance);

    Serial.println(" mm");

    if (

        angle >= 60 &&

        angle <= 120

    ) {

      if (

          distance > 0 &&

          distance <

          OBSTACLE_DISTANCE

      ) {

        obstacleAhead =

            true;

      }

    }

  }

  panServo.write(PAN_CENTER);

  currentPanAngle =

      90;

  delay(250);

  return obstacleAhead;

}

// =====================================================

// FINE SCAN

// =====================================================

int fineScan() {

  int bestAngle =

      90;

  bestScanDistance =

      0;

  Serial.println();

  Serial.println(

      "FINE SCAN"

  );

  // Navigation uses a fast horizontal scan.
  tiltServo.write(TILT_CENTER);
  currentTiltAngle = TILT_CENTER;
  delay(200);

  for (

      int angle = PAN_MIN;

      angle <= PAN_MAX;

      angle += FINE_STEP

  ) {

    server.handleClient();

    if (!autonomousMode) {

      return 90;

    }

    panServo.write(

        angle

    );

    currentPanAngle =

        angle;

    delay(250);

    int distance =

        readDistance();

    Serial.print(angle);

    Serial.print(" deg -> ");

    Serial.print(distance);

    Serial.println(" mm");

    if (

        distance > 0 &&

        distance >

        bestScanDistance

    ) {

      bestScanDistance =

          distance;

      bestAngle =

          angle;

    }

  }

  panServo.write(PAN_CENTER);

  currentPanAngle =

      90;

  delay(250);

  Serial.print(

      "BEST ANGLE: "

  );

  Serial.println(

      bestAngle

  );

  Serial.print(

      "BEST DISTANCE: "

  );

  Serial.print(

      bestScanDistance

  );

  Serial.println(

      " mm"

  );

  return bestAngle;

}

// =====================================================

// AUTONOMOUS STEP

// =====================================================

void autonomousStep() {

  stopMotors();

  bool obstacleAhead =

      coarseScan();

  if (!autonomousMode) {

    stopMotors();

    return;

  }

  // ---------------------------------------------------

  // PATH CLEAR

  // ---------------------------------------------------

  if (!obstacleAhead) {

    Serial.println(

        "CLEAR -> DRIVE FORWARD"

    );

    obstacleCounter =

        0;

    driveForTime(

        forward,

        DRIVE_TIME

    );

    return;

  }

  // ---------------------------------------------------

  // OBSTACLE

  // ---------------------------------------------------

  obstacleCounter++;

  Serial.print(

      "OBSTACLE COUNTER: "

  );

  Serial.println(

      obstacleCounter

  );

  int bestAngle =

      fineScan();

  if (!autonomousMode) {

    stopMotors();

    return;

  }

  // ---------------------------------------------------

  // STRONG STUCK RECOVERY

  // ---------------------------------------------------

  if (

      obstacleCounter >=

      STUCK_LIMIT

  ) {

    Serial.println(

        "STUCK -> STRONG RECOVERY"

    );

    if (

        !driveForTime(

            backward,

            STUCK_REVERSE_TIME

        )

    ) {

      return;

    }

    delay(200);

    if (

        bestAngle < 75

    ) {

      driveForTime(

          turnLeft,

          STUCK_TURN_TIME

      );

    }

    else if (

        bestAngle > 105

    ) {

      driveForTime(

          turnRight,

          STUCK_TURN_TIME

      );

    }

    else {

      lastEscapeLeft =

          !lastEscapeLeft;

      if (lastEscapeLeft) {

        driveForTime(

            turnLeft,

            STUCK_TURN_TIME

        );

      }

      else {

        driveForTime(

            turnRight,

            STUCK_TURN_TIME

        );

      }

    }

    obstacleCounter =

        0;

    return;

  }

  // ---------------------------------------------------

  // NO SAFE DIRECTION

  // ---------------------------------------------------

  if (

      bestScanDistance <

      SAFE_DISTANCE

  ) {

    Serial.println(

        "NO SAFE PATH -> BACKWARD"

    );

    if (

        !driveForTime(

            backward,

            REVERSE_TIME

        )

    ) {

      return;

    }

    delay(200);

    if (

        bestAngle < 90

    ) {

      driveForTime(

          turnLeft,

          ESCAPE_TURN_TIME

      );

    }

    else if (

        bestAngle > 90

    ) {

      driveForTime(

          turnRight,

          ESCAPE_TURN_TIME

      );

    }

    else {

      lastEscapeLeft =

          !lastEscapeLeft;

      if (lastEscapeLeft) {

        driveForTime(

            turnLeft,

            ESCAPE_TURN_TIME

        );

      }

      else {

        driveForTime(

            turnRight,

            ESCAPE_TURN_TIME

        );

      }

    }

    return;

  }

  // ---------------------------------------------------

  // SAFE DIRECTION FOUND

  // ---------------------------------------------------

  if (

      bestAngle < 75

  ) {

    driveForTime(

        turnLeft,

        TURN_TIME

    );

  }

  else if (

      bestAngle > 105

  ) {

    driveForTime(

        turnRight,

        TURN_TIME

    );

  }

  else {

    driveForTime(

        forward,

        DRIVE_TIME

    );

  }

}

// =====================================================

// MAPPING SCAN

// =====================================================

extern bool exploreDriveActive;  // Defined below in the exploration controller.

void handleMapScan() {

  if (autonomousMode || exploreDriveActive) {

    server.send(

        409,

        "text/plain",

        "Stop autonomous mode first"

    );

    return;

  }

  mappingScanActive =

      true;

  stopMotors();
  updateOdometry();
  // Snapshot pose before moving either servo; the rover must stay still.
  const float scanX = odomXmm;
  const float scanY = odomYmm;
  const float scanThetaDeg = odomTheta * 180.0f / PI;

  // 2D map is a horizontal slice: keep tilt fixed at center.
  tiltServo.write(TILT_CENTER);
  currentTiltAngle = TILT_CENTER;
  delay(250);

  Serial.println();

  Serial.println(

      "=============================="

  );

  Serial.println(

      "MAPPING SCAN STARTED"

  );

  Serial.println(

      "=============================="

  );

  String json = "{\"pose\":{\"x\":" + String(scanX,1) +
      ",\"y\":" + String(scanY,1) +
      ",\"theta\":" + String(scanThetaDeg,1) + "},\"points\":[";

  bool firstPoint =

      true;

  for (

      int angle = PAN_MIN;

      angle <= PAN_MAX;

      angle += MAP_SCAN_STEP

  ) {

    panServo.write(

        angle

    );

    currentPanAngle =

        angle;

    delay(

        MAP_SERVO_DELAY

    );

    int distance =

        readDistance();

    Serial.print(angle);

    Serial.print(" deg -> ");

    Serial.print(distance);

    Serial.println(" mm");

    if (!firstPoint) {

      json += ",";

    }

    firstPoint =

        false;

    json += "{";

    json += "\"angle\":";

    json += String(angle);

    json += ",";

    json += "\"distance\":";

    json += String(distance);

    json += "}";

  }

  json += "]}";

  panServo.write(

      90

  );

  currentPanAngle =

      90;

  mappingScanActive =

      false;

  Serial.println(

      "MAPPING SCAN COMPLETE"

  );

  server.send(

      200,

      "application/json",

      json

  );

}

// =====================================================

// WEB HANDLERS

// =====================================================

void handleRoot() {

  server.send(

      200,

      "text/html",

      webpage

  );

}

void handlePan() {

  if (autonomousMode || mappingScanActive || scan3DActive) {
    server.send(409, "text/plain", "Automatic scan active");
    return;
  }

  if (server.hasArg("angle")) {
    int angle = constrain(server.arg("angle").toInt(), 0, 180);
    currentPanAngle = angle;
    panServo.write(angle);
    Serial.print("MANUAL PAN -> ");
    Serial.println(angle);
    server.send(200, "text/plain", "OK");
  }
}

void handleTilt() {

  if (autonomousMode || mappingScanActive || scan3DActive) {
    server.send(409, "text/plain", "Automatic scan active");
    return;
  }

  if (server.hasArg("angle")) {
    int angle = constrain(server.arg("angle").toInt(), 0, 180);
    currentTiltAngle = angle;
    tiltServo.write(angle);
    Serial.print("MANUAL TILT -> ");
    Serial.println(angle);
    server.send(200, "text/plain", "OK");
  }
}

void handle3DScan() {

  if (autonomousMode || mappingScanActive || scan3DActive) {
    server.send(409, "text/plain", "Another automatic mode is active");
    return;
  }

  scan3DActive = true;
  stopMotors();

  Serial.println();
  Serial.println("==============================");
  Serial.println("3D PAN-TILT SCAN STARTED");
  Serial.println("==============================");

  String json = "[";
  bool firstPoint = true;

  // Serpentine scan: every second tilt row reverses the pan direction.
  // This avoids returning all the way across the pan range after each row.
  int row = 0;

  for (int tilt = TILT_MIN; tilt <= TILT_MAX; tilt += SCAN3D_TILT_STEP) {

    tiltServo.write(tilt);
    currentTiltAngle = tilt;
    delay(SCAN3D_SERVO_DELAY);

    bool forwardPan = (row % 2 == 0);
    int panStart = forwardPan ? PAN_MIN : PAN_MAX;
    int panEnd   = forwardPan ? PAN_MAX : PAN_MIN;
    int panStep  = forwardPan ? SCAN3D_PAN_STEP : -SCAN3D_PAN_STEP;

    for (int pan = panStart;
         forwardPan ? (pan <= panEnd) : (pan >= panEnd);
         pan += panStep) {

      panServo.write(pan);
      currentPanAngle = pan;
      delay(SCAN3D_SERVO_DELAY);

      int distance = readDistance();

      Serial.print("Pan ");
      Serial.print(pan);
      Serial.print(" deg, Tilt ");
      Serial.print(tilt);
      Serial.print(" deg -> ");
      Serial.print(distance);
      Serial.println(" mm");

      if (!firstPoint) json += ",";
      firstPoint = false;

      json += "{";
      json += "\"pan\":" + String(pan) + ",";
      json += "\"tilt\":" + String(tilt) + ",";
      json += "\"distance\":" + String(distance);
      json += "}";
    }

    row++;
  }

  json += "]";

  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
  currentPanAngle = PAN_CENTER;
  currentTiltAngle = TILT_CENTER;
  delay(250);

  scan3DActive = false;

  Serial.println("3D PAN-TILT SCAN COMPLETE");
  server.send(200, "application/json", json);
}

// =====================================================

// SETUP

// =====================================================

// =====================================================
// MPU-6050 GYROSCOPE: first comparison test (no sensor fusion yet)
// =====================================================
const uint8_t MPU_ADDR = 0x68;
bool mpuAvailable = false;
float gyroZOffset = 0.0f;
float gyroTotalDegrees = 0.0f;
float gyroSegmentDegrees = 0.0f;
float gyroRateDps = 0.0f;
bool gyroTurning = false;
uint32_t gyroQuietSinceMs = 0;
// Threshold is above typical gyro noise/slow offset drift, but low enough for small turns.
const float GYRO_START_DPS = 1.8f;
const float GYRO_STOP_DPS = 0.9f;
const uint32_t GYRO_STOP_HOLD_MS = 250;
uint32_t gyroLastMicros = 0;
uint32_t gyroLastPrintMs = 0;

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadGyroZ(int16_t &rawZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return false;
  rawZ = (int16_t)((uint16_t(Wire.read()) << 8) | Wire.read());
  return true;
}

void setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU-6050 not found; gyro test disabled");
    return;
  }
  if (!mpuWrite(0x6B, 0x00) || !mpuWrite(0x1B, 0x08) || !mpuWrite(0x1A, 0x03)) {
    Serial.println("MPU-6050 initialization failed");
    return;
  }
  delay(100);
  Serial.println("Keep rover completely STILL: calibrating gyro (~2 s)");
  int32_t sum = 0;
  int count = 0;
  for (int i = 0; i < 400; ++i) {
    int16_t raw;
    if (mpuReadGyroZ(raw)) { sum += raw; ++count; }
    delay(5);
  }
  if (count < 350) {
    Serial.println("MPU-6050 calibration failed");
    return;
  }
  gyroZOffset = float(sum) / count;
  gyroTotalDegrees = 0;
  gyroLastMicros = micros();
  gyroLastPrintMs = millis();
  mpuAvailable = true;
  Serial.println("MPU-6050 ready. Compare gyro and encoder angles.");
}

const float GYRO_LSB_PER_DPS = 65.5f;        // MPU-6050 range +-500 deg/s (reg 0x1B = 0x08)
const uint32_t GYRO_COAST_WINDOW_MS = 350;   // keep integrating while the rover coasts after motor stop
uint16_t gyroSaturationCount = 0;            // >0 while turning = range too small, angle would be too small

void updateGyro() {
  if (!mpuAvailable) return;
  const uint32_t nowUs = micros();
  const float dt = (uint32_t)(nowUs - gyroLastMicros) * 1.0e-6f;
  gyroLastMicros = nowUs;
  int16_t raw;
  if (!mpuReadGyroZ(raw)) return;
  if (raw > 32000 || raw < -32000) gyroSaturationCount++;
  const float prevRate = gyroRateDps;
  gyroRateDps = (float(raw) - gyroZOffset) / GYRO_LSB_PER_DPS;
  bool motorsCommanded = (leftWheelDirection != 0 || rightWheelDirection != 0);
  bool rotatingNow = fabsf(gyroRateDps) >= (gyroTurning ? GYRO_STOP_DPS : GYRO_START_DPS);
  // gyroTurning is now only a status flag for the web UI; it no longer gates the integration.
  if (rotatingNow) {
    if (!gyroTurning) gyroSegmentDegrees = 0.0f;
    gyroTurning = true;
    gyroQuietSinceMs = 0;
  } else if (gyroTurning) {
    if (gyroQuietSinceMs == 0) gyroQuietSinceMs = millis();
    if (millis() - gyroQuietSinceMs >= GYRO_STOP_HOLD_MS) {
      gyroTurning = false;
    }
  }
  static uint32_t lastMotorActiveMs = 0;
  if (motorsCommanded) lastMotorActiveMs = millis();
  const bool motionWindow = motorsCommanded ||
                            (uint32_t)(millis() - lastMotorActiveMs) < GYRO_COAST_WINDOW_MS;
  if (!motionWindow) {
    gyroTurning = false;
    gyroQuietSinceMs = 0;
  }
  // While the motors run (or the rover coasts) ALWAYS integrate - no deadband, so slow
  // turns and the last degrees of braking are not lost. At standstill nothing is
  // integrated, therefore the displayed angle cannot drift while the rover is parked.
  // Trapezoid rule + generous dt limit (WiFi / I2C can delay a loop pass).
  if (motionWindow && dt > 0.0f && dt < 0.3f) {
    float delta = 0.5f * (prevRate + gyroRateDps) * dt;
    gyroSegmentDegrees += delta;
    gyroTotalDegrees += delta;
  }
  // Re-learn the bias at every standstill (temperature drift!). Faster than before.
  if (!motionWindow && fabsf(gyroRateDps) < GYRO_START_DPS) {
    gyroZOffset = 0.98f * gyroZOffset + 0.02f * float(raw);
  }
  if (millis() - gyroLastPrintMs >= 250) {
    gyroLastPrintMs = millis();
    Serial.print("Gyro total [deg]: "); Serial.print(gyroTotalDegrees, 1);
    Serial.print(" | segment [deg]: "); Serial.print(gyroSegmentDegrees, 1);
    Serial.print(" | rate [deg/s]: "); Serial.print(gyroRateDps, 2);
    Serial.print(" | sat: "); Serial.print(gyroSaturationCount);
    Serial.print(" | Encoder [deg]: ");
    Serial.println(odomTotalTheta * 180.0f / PI, 1);
  }
}

void setup() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), onRightEncoder, RISING);


  Serial.begin(

      115200

  );

  // ---------------------------------------------------

  // MOTORS

  // ---------------------------------------------------

  pinMode(

      AIN1,

      OUTPUT

  );

  pinMode(

      AIN2,

      OUTPUT

  );

  pinMode(

      PWMA,

      OUTPUT

  );

  pinMode(

      BIN1,

      OUTPUT

  );

  pinMode(

      BIN2,

      OUTPUT

  );

  pinMode(

      PWMB,

      OUTPUT

  );

  stopMotors();

  // ---------------------------------------------------

  // I2C + VL53L0X

  // ---------------------------------------------------

  Wire.begin(

      SDA_PIN,

      SCL_PIN

  );
  Wire.setClock(400000);

  sensor.setTimeout(

      100

  );

  if (

      !sensor.init()

  ) {

    Serial.println(

        "VL53L0X NOT FOUND!"

    );

    while (1) {

      delay(100);

    }

  }

  sensor.startContinuous(

      50

  );

  setupMPU();

  // ---------------------------------------------------

  // PAN + TILT SERVOS

  // ---------------------------------------------------

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  panServo.attach(PAN_SERVO_PIN, 500, 2400);
  tiltServo.attach(TILT_SERVO_PIN, 500, 2400);

  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
  currentPanAngle = PAN_CENTER;
  currentTiltAngle = TILT_CENTER;

  delay(500);

  // ---------------------------------------------------

  // WIFI

  // ---------------------------------------------------

  WiFi.softAP(

      ssid,

      password

  );

  Serial.println();

  Serial.println(

      "Rover WiFi started!"

  );

  Serial.print(

      "IP: "

  );

  Serial.println(

      WiFi.softAPIP()

  );

  // ---------------------------------------------------

  // WEBSITE

  // ---------------------------------------------------

  server.on(

      "/",

      handleRoot

  );

  // ---------------------------------------------------

  // MANUAL DRIVE

  // ---------------------------------------------------

  server.on(

      "/forward",

      []() {

        if (!autonomousMode && !mappingScanActive && !scan3DActive && !exploreDriveActive) {

          forward();

        }

        server.send(

            200,

            "text/plain",

            "OK"

        );

      }

  );

  server.on(

      "/backward",

      []() {

        if (!autonomousMode && !mappingScanActive && !scan3DActive && !exploreDriveActive) {

          backward();

        }

        server.send(

            200,

            "text/plain",

            "OK"

        );

      }

  );

  server.on(

      "/left",

      []() {

        if (!autonomousMode && !mappingScanActive && !scan3DActive && !exploreDriveActive) {

          turnLeft();

        }

        server.send(

            200,

            "text/plain",

            "OK"

        );

      }

  );

  server.on(

      "/right",

      []() {

        if (!autonomousMode && !mappingScanActive && !scan3DActive && !exploreDriveActive) {

          turnRight();

        }

        server.send(

            200,

            "text/plain",

            "OK"

        );

      }

  );

  server.on(

      "/stop",

      []() {

        stopMotors();

        server.send(

            200,

            "text/plain",

            "OK"

        );

      }

  );

  // ---------------------------------------------------

  // SERVO

  // ---------------------------------------------------

  server.on(

      "/pan",

      handlePan

  );

  server.on(

      "/tilt",

      handleTilt

  );

  server.on(

      "/scan3d",

      handle3DScan

  );

  // ---------------------------------------------------

  // MAPPING

  // ---------------------------------------------------

  server.on("/explore/move", handleExploreMove);
  server.on("/explore/orient", HTTP_GET, handleExploreOrient);
  server.on("/explore/state", handleExploreState);
  server.on("/explore/stop", [](){exploreDriveStop("stopped");server.send(200,"text/plain","stopped");});
  server.on(

      "/mapscan",

      handleMapScan

  );

  // ---------------------------------------------------

  // AUTONOMOUS START

  // ---------------------------------------------------

  server.on(

      "/auto/start",

      []() {

        exploreDriveStop("legacy_auto");
        stopMotors();

        obstacleCounter =

            0;

        autonomousMode =

            true;

        Serial.println(

            "AUTONOMOUS MODE STARTED"

        );

        server.send(

            200,

            "text/plain",

            "AUTO STARTED"

        );

      }

  );

  // ---------------------------------------------------

  // AUTONOMOUS STOP

  // ---------------------------------------------------

  server.on(

      "/auto/stop",

      []() {

        autonomousMode =

            false;

        obstacleCounter =

            0;

        stopMotors();

        panServo.write(PAN_CENTER);
        tiltServo.write(TILT_CENTER);
        currentPanAngle = PAN_CENTER;
        currentTiltAngle = TILT_CENTER;

        Serial.println(

            "AUTONOMOUS MODE STOPPED"

        );

        server.send(

            200,

            "text/plain",

            "AUTO STOPPED"

        );

      }

  );

  server.on("/odometry", HTTP_GET, handleOdometry);
  server.on("/odometry/mode", HTTP_GET, handleOdometryMode);
  server.on("/odometry/reset", HTTP_GET, handleOdometryReset);
  server.begin();

  Serial.println(

      "Webserver started!"

  );

}

// =====================================================
// EXPLORE-AREA waypoint controller (v2)
//  - Motors stay DIGITAL (full power). "Speed" is made by time-sliced pulsing.
//  - Turns are closed-loop to an ABSOLUTE heading (gyro) and are re-checked
//    until the rover has really stopped inside the tolerance (no more blind
//    overshoot that is carried into the straight leg).
//  - Straight legs hold the heading with the gyro (one wheel is pulsed off
//    when the rover drifts) and the gyro is integrated into odometry also
//    while driving, so the map sees the real path.
// =====================================================
bool exploreStraightHeadingLock=false;   // legacy flag, kept for updateOdometry(); always false now
bool exploreDriveActive=false;
uint8_t exploreDrivePhase=0; // 0 idle, 1 align to goal, 2 advance, 3 orient only, 4 reverse, 5 turn away, 7 settle
uint8_t exploreSettleNext=1; // after settling: 1 = move to goal, 3 = orient
bool exploreStraightHint=false;
float exploreGoalX=0, exploreGoalY=0;
uint32_t explorePhaseStarted=0, exploreLastCheck=0;
uint32_t exploreLastProgressMs=0;
float exploreLegStartX=0, exploreLegStartY=0, exploreLegHeading=0; // heading: absolute, rad
float exploreLegDistance=0;
float exploreTargetHeading=0;            // absolute target for turn phases, rad
float exploreTurnLastErrDeg=0;
float exploreTurnBoost=0;                // extra duty if the rover sticks (stiction)
uint32_t exploreTurnSettledSince=0;
bool explorePreviousHeadingMode=true;
String exploreDriveStatus="idle";

const float EXPLORE_GOAL_TOL_MM=23.0f;
const uint32_t EXPLORE_TURN_TIMEOUT_MS=6000;
const uint32_t EXPLORE_FORWARD_TIMEOUT_MS=30000;
const uint32_t EXPLORE_SETTLE_BEFORE_MOVE_MS=350; // rover stands still -> gyro bias is refreshed
const int EXPLORE_STOP_MM=50;   // requested emergency threshold, mm from sensor
const int EXPLORE_SLOW_MM=150;
uint8_t exploreInvalidRangeCount=0;
int exploreLastValidRange=1000;
float exploreRecoveryStartX=0, exploreRecoveryStartY=0;
int exploreAvoidDirection=1;
// Information for the web page about why / where the last drive ended.
float exploreRemainingMm=0;                 // distance to the waypoint when the drive ended
int exploreObstacleRange=0;                 // >0: forward sensor saw an obstacle this close (mm) ...
float exploreObstacleX=0, exploreObstacleY=0, exploreObstacleThetaDeg=0; // ... from this pose
bool exploreFinalLeg=false;                 // last waypoint of the planned path
bool exploreArriveAfterBackoff=false;       // obstacle next to the scan position: back off a bit, then "arrived"
float exploreBackoffMm=160.0f;

// ---- Turn tuning (all values can be tuned on the real rover) ----
const float EXPLORE_TURN_TOL_DEG=3.0f;        // accepted final heading error
const float EXPLORE_TURN_SETTLED_DPS=6.0f;    // "standing still" threshold
const uint32_t EXPLORE_TURN_SETTLE_MS=150;    // must stay inside tolerance this long
const uint32_t TURN_PULSE_PERIOD_MS=50;       // coarse zone: duty cycle period
const float TURN_DUTY_MIN=0.35f;              // minimum duty to overcome stiction
const float TURN_DUTY_MAX=1.00f;
const float TURN_KP_DUTY_PER_DEG=0.02f;       // duty = kp * remaining angle (50 deg -> 100 %)
const float TURN_COAST_S=0.08f;               // estimated coasting time after switching the motors off
const float TURN_FINE_ZONE_DEG=12.0f;         // below this: short "nudge" pulses
const uint32_t TURN_FINE_PERIOD_MS=90;
const uint32_t TURN_FINE_ON_MS=14;
// ---- Heading hold on straight legs ----
const uint32_t HOLD_PERIOD_MS=40;
const float HOLD_KP_PER_DEG=0.08f;            // 12.5 deg error -> full correction
const float HOLD_MAX_CORR=0.6f;               // slow wheel runs at least 40 % of the time

float exploreWrap(float angle){
  while(angle>PI) angle-=2.0f*PI;
  while(angle< -PI) angle+=2.0f*PI;
  return angle;
}

// Same digital pin pattern as forward()/turnLeft()/..., but per wheel (-1,0,+1).
// A wheel that is switched OFF coasts and KEEPS its last direction sign, so the
// encoder ticks of the coasting wheel are still counted with the right sign.
void driveWheels(int left,int right){
  const int dl=(left!=0)?left:(int)leftWheelDirection;
  const int dr=(right!=0)?right:(int)rightWheelDirection;
  setWheelDirections(dl,dr);
  digitalWrite(AIN1,left>0?HIGH:LOW);
  digitalWrite(AIN2,left<0?HIGH:LOW);
  digitalWrite(BIN1,right>0?HIGH:LOW);
  digitalWrite(BIN2,right<0?HIGH:LOW);
  digitalWrite(PWMA,left!=0?HIGH:LOW);
  digitalWrite(PWMB,right!=0?HIGH:LOW);
}

void exploreDriveStop(const String &why){
  stopMotors();
  exploreRemainingMm=hypotf(exploreGoalX-odomXmm,exploreGoalY-odomYmm);
  exploreStraightHeadingLock=false;
  useGyroHeading=explorePreviousHeadingMode;
  exploreDriveActive=false; exploreDrivePhase=0; exploreDriveStatus=why;
  exploreTurnSettledSince=0; exploreTurnBoost=0; exploreArriveAfterBackoff=false;
  Serial.println("EXPLORE: "+why+" (remaining "+String(exploreRemainingMm,0)+" mm)");
}

void exploreBeginTurn(uint8_t phase,const char* status){
  const uint32_t now=millis();
  exploreDrivePhase=phase; exploreDriveStatus=status;
  explorePhaseStarted=now; exploreLastProgressMs=now;
  exploreTurnLastErrDeg=exploreWrap(exploreTargetHeading-odomTheta)*180.0f/PI;
  exploreTurnSettledSince=0; exploreTurnBoost=0;
}

void exploreStartStraight(uint32_t now){
  stopMotors();
  exploreDrivePhase=2; exploreDriveStatus="straight";
  explorePhaseStarted=now; exploreLastCheck=0;
  exploreInvalidRangeCount=0; exploreLastValidRange=1000;
  exploreLegStartX=odomXmm; exploreLegStartY=odomYmm;
  exploreLegDistance=hypotf(exploreGoalX-odomXmm,exploreGoalY-odomYmm);
}

// Called when the rover stands still: compute the turn from the CURRENT pose.
void exploreBeginMove(){
  const uint32_t now=millis();
  updateOdometry();
  const float dx=exploreGoalX-odomXmm, dy=exploreGoalY-odomYmm;
  if(hypotf(dx,dy)<EXPLORE_GOAL_TOL_MM){exploreDriveStop("arrived");return;}
  exploreLegHeading=atan2f(dy,dx);
  exploreTargetHeading=exploreLegHeading;
  const float errDeg=fabsf(exploreWrap(exploreLegHeading-odomTheta))*180.0f/PI;
  Serial.printf("EXPLORE new waypoint: %.0f mm, heading error %.1f deg\n",hypotf(dx,dy),errDeg);
  if(exploreStraightHint && errDeg<15.0f){exploreStartStraight(now);return;}
  exploreBeginTurn(1,"turning");
}

// One step of the closed-loop turn. Returns true when the rover has settled
// inside the tolerance. Calls exploreDriveStop() itself on a stall.
bool exploreTurnStep(uint32_t now,float targetRad){
  const float errDeg=exploreWrap(targetRad-odomTheta)*180.0f/PI;   // + = target is to the left
  const float rate=gyroRateDps;                                    // + = turning left
  if(fabsf(errDeg)<=EXPLORE_TURN_TOL_DEG && fabsf(rate)<EXPLORE_TURN_SETTLED_DPS){
    driveWheels(0,0);
    if(exploreTurnSettledSince==0)exploreTurnSettledSince=now;
    if(now-exploreTurnSettledSince>=EXPLORE_TURN_SETTLE_MS){
      stopMotors();exploreTurnSettledSince=0;return true;
    }
    return false;
  }
  exploreTurnSettledSince=0;
  // Stall watchdog (stiction): only far away from the target.
  if(fabsf(errDeg-exploreTurnLastErrDeg)>1.5f){
    exploreTurnLastErrDeg=errDeg;exploreLastProgressMs=now;
  }else if(fabsf(errDeg)>10.0f){
    const uint32_t stalled=now-exploreLastProgressMs;
    if(stalled>1800){exploreDriveStop("no_turn_progress");return false;}
    exploreTurnBoost=fmaxf(exploreTurnBoost,fminf(0.45f,0.12f*float(stalled/300)));
  }
  // Predict where the rover ends up if the motors are switched off now.
  const float predicted=errDeg-rate*TURN_COAST_S;
  if(fabsf(predicted)<=EXPLORE_TURN_TOL_DEG || predicted*errDeg<=0.0f){
    driveWheels(0,0);              // will coast into the tolerance (or be pushed back next cycle)
    return false;
  }
  bool on;
  if(fabsf(predicted)<TURN_FINE_ZONE_DEG){
    on=(float)(now%TURN_FINE_PERIOD_MS) < (float)TURN_FINE_ON_MS+exploreTurnBoost*40.0f;
  }else{
    float duty=TURN_KP_DUTY_PER_DEG*fabsf(predicted)+exploreTurnBoost;
    if(duty<TURN_DUTY_MIN)duty=TURN_DUTY_MIN;
    if(duty>TURN_DUTY_MAX)duty=TURN_DUTY_MAX;
    on=(float)(now%TURN_PULSE_PERIOD_MS) < duty*(float)TURN_PULSE_PERIOD_MS;
  }
  if(!on){driveWheels(0,0);return false;}
  if(predicted>0.0f)driveWheels(-1,1);   // left turn  (same wheel signs as turnLeft())
  else              driveWheels(1,-1);   // right turn (same wheel signs as turnRight())
  return false;
}

// Straight driving with gyro heading hold: the wheel on the side we want to
// turn towards is pulsed off a fraction of the time.
void exploreHoldDrive(float desiredHeading){
  const uint32_t now=millis();
  const float errDeg=exploreWrap(desiredHeading-odomTheta)*180.0f/PI;   // + = target is to the left
  float corr=HOLD_KP_PER_DEG*errDeg;
  if(corr>HOLD_MAX_CORR)corr=HOLD_MAX_CORR;
  if(corr<-HOLD_MAX_CORR)corr=-HOLD_MAX_CORR;
  float leftDuty=1.0f,rightDuty=1.0f;
  if(corr>0.0f)leftDuty=1.0f-corr; else rightDuty=1.0f+corr;
  const float ph=(float)(now%HOLD_PERIOD_MS);
  driveWheels(ph<leftDuty*(float)HOLD_PERIOD_MS?1:0, ph<rightDuty*(float)HOLD_PERIOD_MS?1:0);
}

void exploreDriveUpdate(){
  if(!exploreDriveActive)return;
  if(autonomousMode||mappingScanActive||scan3DActive){exploreDriveStop("interrupted");return;}
  const uint32_t now=millis();

  if(exploreDrivePhase==7){                       // stand still, let the gyro bias settle
    if(now-explorePhaseStarted<EXPLORE_SETTLE_BEFORE_MOVE_MS)return;
    if(exploreSettleNext==3)exploreBeginTurn(3,"orienting");
    else exploreBeginMove();
    return;
  }

  if(exploreDrivePhase==1||exploreDrivePhase==3){ // closed-loop turn to absolute heading
    if(!mpuAvailable){exploreDriveStop("gyro_unavailable");return;}
    const bool done=exploreTurnStep(now,exploreTargetHeading);
    if(!exploreDriveActive)return;
    if(done){
      Serial.printf("EXPLORE turn finished, heading error %.1f deg\n",
                    exploreWrap(exploreTargetHeading-odomTheta)*180.0f/PI);
      if(exploreDrivePhase==3){exploreDriveStop("arrived");return;}
      exploreStartStraight(now);
      return;
    }
    if(now-explorePhaseStarted>EXPLORE_TURN_TIMEOUT_MS){
      const float errAbs=fabsf(exploreWrap(exploreTargetHeading-odomTheta))*180.0f/PI;
      if(errAbs<8.0f){                           // good enough, continue
        stopMotors();
        if(exploreDrivePhase==3){exploreDriveStop("arrived");return;}
        exploreStartStraight(now);
      }else exploreDriveStop("turn_timeout");
    }
    return;
  }

  if(exploreDrivePhase==2){                       // straight leg with heading hold
    const float travelled=hypotf(odomXmm-exploreLegStartX,odomYmm-exploreLegStartY);
    const float remX=exploreGoalX-odomXmm, remY=exploreGoalY-odomYmm;
    const float remaining=hypotf(remX,remY);
    if(travelled>=exploreLegDistance-EXPLORE_GOAL_TOL_MM || remaining<EXPLORE_GOAL_TOL_MM){
      exploreDriveStop("arrived");return;
    }
    if(now-explorePhaseStarted>EXPLORE_FORWARD_TIMEOUT_MS){exploreDriveStop("forward_timeout");return;}
    if(now-exploreLastCheck>=60){
      exploreLastCheck=now;
      int d=readDistance();
      // Invalid/out-of-range readings are ignored; only a valid measurement may slow/stop.
      if(d>0 && d<=8000){
        exploreLastValidRange=d;
        if(d<=EXPLORE_STOP_MM){
          stopMotors();
          // Remember what was seen where: the web page enters it into the map.
          exploreObstacleRange=d;
          exploreObstacleX=odomXmm;exploreObstacleY=odomYmm;
          exploreObstacleThetaDeg=odomTheta*180.0f/PI;
          exploreRecoveryStartX=odomXmm;exploreRecoveryStartY=odomYmm;
          // Obstacle right next to the scan position: that is not a failure. Back off a
          // little (room to turn) and report "arrived" instead of running away from the target.
          exploreArriveAfterBackoff=(remaining<120.0f || (exploreFinalLeg && remaining<250.0f));
          exploreBackoffMm=exploreArriveAfterBackoff?100.0f:160.0f;
          exploreDrivePhase=4;
          exploreDriveStatus=exploreArriveAfterBackoff?"backing_off":"reversing";
          explorePhaseStarted=now;
          return;
        }
      }
    }
    if(now-explorePhaseStarted>650 && travelled<5.0f){exploreDriveStop("no_forward_progress");return;}
    // Steer gently towards the goal (limited), freeze the heading right before arriving.
    float desired=exploreLegHeading;
    if(remaining>150.0f){
      float dev=exploreWrap(atan2f(remY,remX)-exploreLegHeading);
      if(dev>0.35f)dev=0.35f;
      if(dev<-0.35f)dev=-0.35f;
      desired=exploreLegHeading+dev;
    }
    if(exploreLastValidRange<=EXPLORE_SLOW_MM){   // slow approach: short pulses
      if((now%220)<75)exploreHoldDrive(desired);
      else driveWheels(0,0);
    }else exploreHoldDrive(desired);
    return;
  }

  if(exploreDrivePhase==4){
    // Back up 16 cm before turning, so the front and sides clear the obstacle.
    if(hypotf(odomXmm-exploreRecoveryStartX,odomYmm-exploreRecoveryStartY)>=exploreBackoffMm ||
       now-explorePhaseStarted>=3000){
      stopMotors();
      if(exploreArriveAfterBackoff){exploreDriveStop("arrived");return;}
      exploreAvoidDirection=-exploreAvoidDirection;
      exploreTargetHeading=exploreWrap(odomTheta+(float)exploreAvoidDirection*80.0f*PI/180.0f);
      exploreBeginTurn(5,"avoiding_turn");
      return;
    }
    backward();
    return;
  }

  if(exploreDrivePhase==5){                       // turn ~80 deg away, closed-loop
    if(!mpuAvailable){exploreDriveStop("gyro_unavailable");return;}
    const bool done=exploreTurnStep(now,exploreTargetHeading);
    if(!exploreDriveActive)return;
    if(done){exploreDriveStop("avoided");return;}
    if(now-explorePhaseStarted>EXPLORE_TURN_TIMEOUT_MS)exploreDriveStop("avoided"); // best effort
  }
}

void handleExploreMove(){
  if(exploreDriveActive||autonomousMode||mappingScanActive||scan3DActive){server.send(409,"text/plain","Busy");return;}
  if(!server.hasArg("x")||!server.hasArg("y")){server.send(400,"text/plain","Missing target");return;}
  float x=server.arg("x").toFloat(),y=server.arg("y").toFloat();
  updateOdometry();
  float dx=x-odomXmm,dy=y-odomYmm;
  float dist=hypotf(dx,dy);
  if(!isfinite(x)||!isfinite(y)||dist>1200.0f||dist<5.0f){server.send(400,"text/plain","Waypoint must be 5..1200 mm away");return;}
  if(!mpuAvailable){server.send(409,"text/plain","Gyroscope unavailable");return;}
  explorePreviousHeadingMode=useGyroHeading;
  useGyroHeading=true; // heading always comes from the gyro, also while driving straight
  exploreStraightHeadingLock=false;
  exploreGoalX=x;exploreGoalY=y;
  // "straight" is only a hint: if the heading is already within ~15 deg, the turn is skipped.
  exploreStraightHint=server.hasArg("straight") && server.arg("straight")=="1";
  exploreFinalLeg=server.hasArg("final") && server.arg("final")=="1";
  exploreObstacleRange=0; exploreArriveAfterBackoff=false;
  exploreDriveActive=true;
  if(exploreStraightHint){
    exploreBeginMove();                           // chained leg: no extra settle pause
  }else{
    stopMotors();
    exploreDrivePhase=7;exploreSettleNext=1;
    explorePhaseStarted=millis();exploreDriveStatus="settling";
  }
  server.send(200,"text/plain","moving");
}

// Rotate in place to an ABSOLUTE heading (degrees, odometry frame).
void handleExploreOrient(){
  if(exploreDriveActive||autonomousMode||mappingScanActive||scan3DActive){
    server.send(409,"text/plain","Busy");return;
  }
  if(!server.hasArg("theta")||!mpuAvailable){
    server.send(400,"text/plain","Missing theta or gyro unavailable");return;
  }
  updateOdometry();
  explorePreviousHeadingMode=useGyroHeading;
  useGyroHeading=true;
  exploreStraightHeadingLock=false;
  exploreTargetHeading=exploreWrap(server.arg("theta").toFloat()*PI/180.0f);
  exploreGoalX=odomXmm;exploreGoalY=odomYmm;
  exploreStraightHint=false;
  exploreFinalLeg=false; exploreObstacleRange=0; exploreArriveAfterBackoff=false;
  stopMotors();
  exploreDriveActive=true;
  exploreDrivePhase=7;exploreSettleNext=3;
  explorePhaseStarted=millis();exploreDriveStatus="settling";
  server.send(200,"text/plain","orienting");
}

void handleExploreState(){
  server.send(200,"application/json",String("{\"active\":")+(exploreDriveActive?"true":"false")+
    ",\"status\":\""+exploreDriveStatus+"\""+
    ",\"rem\":"+String(exploreRemainingMm,0)+
    ",\"range\":"+String(exploreObstacleRange)+
    ",\"ox\":"+String(exploreObstacleX,1)+
    ",\"oy\":"+String(exploreObstacleY,1)+
    ",\"oth\":"+String(exploreObstacleThetaDeg,1)+"}");
}

// =====================================================

// LOOP

// =====================================================

void loop() {
  updateOdometry();
  updateGyro();


  server.handleClient();
  exploreDriveUpdate();

  if (

      autonomousMode

  ) {

    autonomousStep();

  }

}
