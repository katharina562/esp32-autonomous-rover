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
// Effective wheelbase for turns. Start with measured 125 mm, then calibrate.
// If a physical 360-degree turn displays A degrees (unwrapped), use:
// NEW_WHEEL_BASE_MM = 125.0f * abs(A) / 360.0f;
const float WHEEL_BASE_MM = 104.0f;
const float MM_PER_TICK = PI * WHEEL_DIAMETER_MM / TICKS_PER_REV;

// Assumes motor A drives LEFT wheel, motor B drives RIGHT wheel.
// Verify physical mapping before trusting heading sign.
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t pendingSignedLeft = 0, pendingSignedRight = 0;
volatile uint32_t pendingAbsoluteLeft = 0, pendingAbsoluteRight = 0;
volatile int8_t leftWheelDirection = 0, rightWheelDirection = 0;

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

void updateOdometry() {
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

  odomLeftTotal += absoluteL;
  odomRightTotal += absoluteR;
  float dL = float(signedL) * MM_PER_TICK;
  float dR = float(signedR) * MM_PER_TICK;
  odomLeftMm += dL;
  odomRightMm += dR;

  float dS = (dL + dR) * 0.5f;
  float dTheta = (dR - dL) / WHEEL_BASE_MM;
  float midTheta = odomTheta + 0.5f * dTheta;
  float travel = (fabsf(dTheta) < 0.00001f)
      ? dS : dS * sinf(0.5f * dTheta) / (0.5f * dTheta);
  odomXmm += travel * cosf(midTheta);
  odomYmm += travel * sinf(midTheta);
  odomTheta += dTheta;
  odomTotalTheta += dTheta;
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

void handleOdometry() {
  updateOdometry();
  String json = "{";
  json += "\"x\":" + String(odomXmm, 1);
  json += ",\"y\":" + String(odomYmm, 1);
  json += ",\"theta\":" + String(odomTheta * 180.0f / PI, 1);
  json += ",\"totalTheta\":" + String(odomTotalTheta * 180.0f / PI, 1);
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

void handleOdometryReset() {
  updateOdometry();
  odomXmm = odomYmm = odomTheta = odomTotalTheta = odomDistanceMm = 0;
  odomLeftMm = odomRightMm = 0;
  odomLeftTotal = odomRightTotal = 0;
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
  <p>Heading: <b id="odomHeading">0°</b> &nbsp; Distance: <b id="odomDistance">0 mm</b></p>
  <p><strong>Calibration: Total turn (unwrapped): <b id="odomTotalTheta">0°</b></strong></p>
  <p>Estimated wheelbase after exactly one real 360° turn: <b id="odomCalibratedWheelbase">—</b></p>
  <p style="font-size:13px;">Reset position, turn exactly 360° in one direction, then read the total turn and estimated wheelbase. Repeat left and right several times. Edit WHEEL_BASE_MM at the top of the sketch after averaging reliable tests.</p>
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

    <p>Registered scans: <b id="mapScanCount">0</b></p>
    <button id="resetGlobalMap" class="scanButton">RESET MAP + POSITION</button>
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
    document.getElementById("odomXY").textContent = o.x.toFixed(1) + ", " + o.y.toFixed(1);
    document.getElementById("odomHeading").textContent = o.theta.toFixed(1) + "°";
    document.getElementById("odomTotalTheta").textContent = o.totalTheta.toFixed(1) + "°";
    document.getElementById("odomCalibratedWheelbase").textContent =
      (Math.abs(o.totalTheta) > 1 ? (o.wheelbase * Math.abs(o.totalTheta) / 360).toFixed(1) + " mm" : "—");
    document.getElementById("odomDistance").textContent = o.distance.toFixed(1) + " mm";
    document.getElementById("odomLeft").textContent = o.leftTicks;
    document.getElementById("odomRight").textContent = o.rightTicks;
    document.getElementById("odomLeftMm").textContent = o.leftWheelMm.toFixed(1) + " mm";
    document.getElementById("odomRightMm").textContent = o.rightWheelMm.toFixed(1) + " mm";
  } catch (e) { /* scan may temporarily block HTTP server */ }
}
document.getElementById("odomReset").addEventListener("click", async () => {
  try { await fetch("/odometry/reset"); await refreshOdometry(); } catch (e) {}
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
let scanCount = 0;
let roverTrail = [{x:0,y:0,theta:0}];
let mapBusy = false;

function createEmptyGrid() {
  occupancyGrid = Array.from({length:GRID_ROWS}, () => new Float32Array(GRID_COLS));
  observationGrid = Array.from({length:GRID_ROWS}, () => new Uint8Array(GRID_COLS));
}
function updateCell(col,row,increment) {
  if (col < 0 || row < 0 || col >= GRID_COLS || row >= GRID_ROWS) return;
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
  const start = worldToCell(roverRight,roverForward);
  const end = worldToCell(roverRight+rayRight*distanceMM,
                          roverForward+rayForward*distanceMM);
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
}

function drawScan(result) {
  if (!result || !Array.isArray(result.points) || !result.pose) throw Error("Invalid scan data");
  const data=result.points, pose=result.pose;
  // Each scan is registered using the pose captured when the scan started.
  data.forEach(p=>addSensorRay(p.angle,p.distance,pose));
  // No fabricated obstacle endpoints between real sensor measurements.
  // Contradictory scans can now revise cells instead of permanently locking them.
  scanCount++;
  currentPose=pose;
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

document.getElementById("resetGlobalMap").addEventListener("click",async()=>{
  if(mapBusy || autonomous)return;
  try {
    const r=await fetch("/odometry/reset");
    if(!r.ok)throw Error("Reset failed");
    currentPose={x:0,y:0,theta:0};roverTrail=[{...currentPose}];
    scanCount=0;createEmptyGrid();drawOccupancyGrid();
    document.getElementById("mapScanCount").textContent="0";
    scanStatus.textContent="Map and odometry reset";
  }catch(e){scanStatus.textContent="Reset failed";}
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
let viewYaw = -0.45;
let viewPitch = -0.35;
let viewZoom = 0.13;
const active3DPointers = new Map();
let lastPointerX = 0;
let lastPointerY = 0;
let lastPinchDistance = 0;

function convert3DScan(data) {
    scan3DPoints = [];
    data.forEach(p => {
        if (p.distance <= 0 || p.distance > 2000) return;
        const az = (p.pan - 90) * Math.PI / 180.0;
        const el = (p.tilt - 90) * Math.PI / 180.0;
        const horizontal = p.distance * Math.cos(el);
        scan3DPoints.push({
            x: horizontal * Math.sin(az),
            y: horizontal * Math.cos(az),
            z: p.distance * Math.sin(el),
            distance: p.distance
        });
    });
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

function draw3DPointCloud(data) {
    if (data) convert3DScan(data);
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
        return q;
    });
    projected.sort((a,b) => b.depth - a.depth);

    projected.forEach(p => {
        const radius = Math.max(1.5, Math.min(4, 2.4 * p.scale));
        ctx3D.fillStyle = "#6ee7ff";
        ctx3D.beginPath();
        ctx3D.arc(p.x, p.y, radius, 0, 2 * Math.PI);
        ctx3D.fill();
    });

    const rover = project3D({x:0,y:0,z:0}, w, h);
    ctx3D.fillStyle = "#ff4d4d";
    ctx3D.beginPath();
    ctx3D.arc(rover.x, rover.y, 5, 0, 2 * Math.PI);
    ctx3D.fill();
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

scan3DButton.addEventListener("click", function() {
    if (autonomous) {
        scan3DStatus.innerText = "Stop autonomous mode first.";
        return;
    }
    scan3DButton.disabled = true;
    scan3DStatus.innerText = "3D scanning...";
    fetch("/scan3d")
    .then(response => {
        if (!response.ok) throw new Error("3D scan failed");
        return response.json();
    })
    .then(data => {
        scan3DStatus.innerText = "3D scan complete: " + data.length + " points";
        draw3DPointCloud(data);
        console.log("3D scan points:", data);
        scan3DButton.disabled = false;
    })
    .catch(error => {
        scan3DStatus.innerText = "3D scan error";
        console.log(error);
        scan3DButton.disabled = false;
    });
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

// TIMED MOVEMENT

// =====================================================

bool driveForTime(

    void (*movement)(),

    int duration

) {

  movement();

  unsigned long start =

      millis();

  while (

      millis() - start <

      (unsigned long)duration

  ) {

    server.handleClient();
    updateOdometry();

    if (!autonomousMode) {

      stopMotors();

        tiltServo.write(TILT_CENTER);
        currentTiltAngle = TILT_CENTER;

      return false;

    }

    delay(10);

  }

  stopMotors();

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

void handleMapScan() {

  if (autonomousMode) {

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

        if (

            !autonomousMode

        ) {

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

        if (

            !autonomousMode

        ) {

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

        if (

            !autonomousMode

        ) {

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

        if (

            !autonomousMode

        ) {

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
  server.on("/odometry/reset", HTTP_GET, handleOdometryReset);
  server.begin();

  Serial.println(

      "Webserver started!"

  );

}

// =====================================================

// LOOP

// =====================================================

void loop() {
  updateOdometry();


  server.handleClient();

  if (

      autonomousMode

  ) {

    autonomousStep();

  }

}