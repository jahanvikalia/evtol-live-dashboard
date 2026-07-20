/*********************  ESP32 Live Dashboard (Merged)  *********************
 * I2C (MPU6050): SDA=19, SCL=18, addr=0x68
 * DS18B20 (Dallas/OneWire): GPIO21  (3.3V, GND, 4.7k pull-up recommended)
 * IR sensors (digital): 33, 32, 27, 26, 25   (1 = white/no line, 0 = black)
 * LEDs: RED=14, GREEN=12, BLUE=13
 * Buzzer: 23 (active buzzer)
 * Open browser to the printed IP (or AP fallback "ESP32-Dashboard" if WiFi fails)
 ***************************************************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>   // *** Library Manager -> search "TinyGPS++" by Mikal Hart -> Install ***
#include <math.h>

// ----------- NEO-6M GPS on UART2 -----------
// GPS TX  ->  ESP32 GPIO16 (RX2)
// GPS RX  ->  ESP32 GPIO17 (TX2)
// GPS VCC ->  3.3 V     GPS GND -> GND
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD   9600

TinyGPSPlus    gps;
HardwareSerial gpsSerial(2);  // UART2

float gps_lat   = NAN;
float gps_lng   = NAN;
bool  gps_valid = false;

// ---------------- WiFi ----------------
// NOTE: credentials moved out of source. Create a "secrets.h" file (gitignored)
// containing:
//   #define WIFI_SSID     "YourNetworkName"
//   #define WIFI_PASSWORD "YourPassword"
#include "secrets.h"
const char* SSID     = WIFI_SSID;
const char* PASSWORD = WIFI_PASSWORD;

// ---------------- Web server ----------------
WebServer server(80);

// ----------- MPU6050 -----------
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

// ----------- IR Sensor Pins -----------
#define IR1 33
#define IR2 32
#define IR3 27
#define IR4 26
#define IR5 25

// ----------- Buzzer and RGB LED Pins -----------
#define BUZZER     23
#define RED_LED    14
#define GREEN_LED  12
#define BLUE_LED   13

// ----------- Dallas Temperature Sensor -----------
#define ONE_WIRE_BUS 21
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// ----------- Movement thresholds (raw units) -----------
const long STILL_THRESH = 17000;
const long FAST_THRESH  = 25000;

// ----------- Speed / velocity state -----------
const float LSB_PER_G = 16384.0f;
float g_baseline = 1.0f;
unsigned long lastIMUms_forSpeed = 0;
float speed_mps = 0.0f;
float vx_mps    = 0.0f;
float vy_mps    = 0.0f;

// ================== HTML ==================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<title>ESP32 eVTOL Dashboard</title>
<style>
  :root {
    --bg:#0b1020; --card:#141a2e; --ink:#e7e9f3; --muted:#94a3b8;
    --ok:#22c55e; --warn:#f59e0b; --alert:#ef4444;
    --accent:#60a5fa; --accent2:#34d399; --accent3:#f472b6;
  }
  * { box-sizing:border-box; }
  body {
    margin:0;
    font-family: ui-sans-serif, system-ui, Segoe UI, Roboto, Helvetica, Arial;
    background: radial-gradient(1200px 600px at 10% -10%, #1e293b 0%, #0b1020 50%, #050812 100%);
    color: var(--ink);
  }
  header {
    display:flex; align-items:center; justify-content:space-between; padding:16px 20px;
    backdrop-filter:blur(6px); position:sticky; top:0; z-index:2;
    background:linear-gradient(180deg,rgba(10,14,26,0.85),rgba(10,14,26,0));
    border-bottom:1px solid rgba(255,255,255,0.06);
  }
  header .ip { color:var(--muted); font-size:14px; }
  .wrap {
    padding:20px; max-width:1200px; margin:0 auto;
    display:grid; gap:20px;
    grid-template-columns:repeat(auto-fit, minmax(290px, 1fr));
  }
  .card {
    background:linear-gradient(180deg,rgba(255,255,255,0.03),rgba(255,255,255,0.01));
    border:1px solid rgba(255,255,255,0.08);
    border-radius:16px; padding:18px;
    box-shadow:0 10px 30px rgba(0,0,0,0.35),inset 0 1px 0 rgba(255,255,255,0.05);
  }
  .card h3 { margin:0 0 12px; font-weight:600; letter-spacing:0.3px; }
  .muted { color:var(--muted); }
  .row { display:flex; gap:16px; align-items:center; }
  .thermo {
    width:80px; height:220px; border-radius:40px;
    background:linear-gradient(160deg,#0f172a,#0b1020);
    border:1px solid rgba(255,255,255,0.08);
    position:relative; padding:8px; margin:6px auto 8px;
    box-shadow:inset 0 0 0 1px rgba(255,255,255,0.06),0 8px 18px rgba(2,6,23,0.6);
  }
  .thermo .tube {
    position:absolute; left:50%; transform:translateX(-50%);
    bottom:14px; width:20px; height:160px; border-radius:10px;
    background:linear-gradient(180deg,#0b1020,#0b1020);
    border:1px solid rgba(255,255,255,0.06); overflow:hidden;
  }
  .thermo .fill {
    position:absolute; bottom:0; left:0; width:100%; height:0%;
    background:linear-gradient(180deg,#34d399,#16a34a);
    transition:height 0.25s ease;
  }
  .thermo .bulb {
    position:absolute; bottom:-4px; left:50%; transform:translateX(-50%);
    width:54px; height:54px; border-radius:50%;
    background:radial-gradient(circle at 30% 30%,#38bdf8,#2563eb);
    box-shadow:inset 0 0 16px rgba(255,255,255,0.15),0 0 30px rgba(96,165,250,0.35);
    border:1px solid rgba(255,255,255,0.08);
  }
  .temp-read { font-size:38px; font-weight:700; text-align:center; margin-top:6px; }
  .sky {
    position:relative; height:240px; border-radius:14px; overflow:hidden;
    background:linear-gradient(180deg,#0a0f1e 0%,#0e172f 60%,#0b1020 100%);
    border:1px solid rgba(255,255,255,0.06);
  }
  .horizon { position:absolute; left:0; right:0; bottom:48px; height:2px; background:rgba(255,255,255,0.06); }
  .vtol { position:absolute; width:120px; height:60px; left:0; bottom:74px; transition:transform 0.12s linear; }
  .vtol svg { width:100%; height:100%; }
  .vtol {
  position:absolute; width:120px; height:60px;
  bottom:74px;
  transition: transform 0.15s ease, left 0.15s ease;  /* add left */
}
  .speed-read { font-size:28px; font-weight:700; margin-top:10px; }
  .pad { display:grid; grid-template-columns:repeat(5,1fr); gap:10px; margin-top:8px; }
  .ir {
    height:28px; border-radius:999px; display:flex; align-items:center; justify-content:center;
    background:linear-gradient(180deg,rgba(255,255,255,0.05),rgba(255,255,255,0.03));
    border:1px solid rgba(255,255,255,0.08);
    color:#e2e8f0; font-weight:600;
  }
  .ir.ok  { background:linear-gradient(180deg,rgba(34,197,94,0.25),rgba(34,197,94,0.12)); }
  .ir.bad { background:linear-gradient(180deg,rgba(239,68,68,0.28),rgba(239,68,68,0.12)); }
  .status {
    margin-top:12px; font-size:15px; font-weight:600;
    padding:10px; border-radius:10px; text-align:center;
    border:1px solid rgba(255,255,255,0.08);
  }
  .status.safe   { color:#16a34a; background:rgba(22,163,74,0.12); }
  .status.danger { color:#ef4444; background:rgba(239,68,68,0.12); }
  .banner {
    margin:-10px auto 0; padding:8px 12px; width:fit-content; border-radius:999px;
    border:1px solid rgba(255,255,255,0.12); font-weight:700; letter-spacing:0.4px;
  }
  .banner.ok    { color:var(--ok);    background:rgba(34,197,94,0.15); }
  .banner.alert { color:var(--alert); background:rgba(239,68,68,0.15); }
</style>
</head>
<body>
<header>
  <div style="display:flex;align-items:center;gap:10px;">
    <svg width="26" height="26" viewBox="0 0 24 24" fill="none">
      <path d="M2 12a10 10 0 1 1 20 0" stroke="#60a5fa" stroke-width="2" stroke-linecap="round"/>
    </svg>
    <div>
      <div style="font-weight:700">ESP32 eVTOL Dashboard</div>
      <div class="ip" id="ip">IP: —</div>
    </div>
  </div>
  <div id="globBanner" class="banner ok">✅ Normal</div>
</header>

<div class="wrap">

  <!-- Temperature -->
  <div class="card">
    <h3>Temperature</h3>
    <div class="row" style="justify-content:center;">
      <div class="thermo">
        <div class="tube"><div id="fill" class="fill" style="height:0%"></div></div>
        <div class="bulb"></div>
      </div>
    </div>
    <div id="tempRead" class="temp-read">--.- °C</div>
    <div class="muted" style="text-align:center;">DS18B20 (live)</div>
  </div>

  <!-- GPS Map -->
  <div class="card">
    <h3>GPS Location</h3>
    <div id="map" style="height:250px;border-radius:12px;"></div>
  </div>

  <!-- Movement / Speed -->
  <div class="card">
    <h3>Movement & Speed</h3>
    <div class="sky" id="sky">
      <div class="horizon"></div>
      <div class="vtol" id="vtol">
        <svg viewBox="0 0 200 100">
          <ellipse cx="100" cy="55" rx="46" ry="14" fill="#334155"/>
          <rect x="40" y="40" width="120" height="12" rx="6" fill="#64748b"/>
          <circle cx="40" cy="46" r="16" fill="#0ea5e9" opacity="0.75">
            <animate attributeName="r" values="14;16;14" dur="0.4s" repeatCount="indefinite"/>
          </circle>
          <circle cx="160" cy="46" r="16" fill="#0ea5e9" opacity="0.75">
            <animate attributeName="r" values="14;16;14" dur="0.4s" repeatCount="indefinite"/>
          </circle>
          <rect x="92" y="28" width="16" height="18" rx="4" fill="#60a5fa"/>
        </svg>
      </div>
    </div>
    <div id="speedRead" class="speed-read">Speed: --.- m/s</div>
    <div class="muted">State: <span id="moveState">—</span></div>
  </div>

  <!-- IR Landing Safety -->
  <div class="card">
    <h3>Landing Safety (IR)</h3>
    <div class="muted">1 = clear surface, 0 = black/line detected</div>
    <div class="pad">
      <div id="ir1" class="ir">IR1</div>
      <div id="ir2" class="ir">IR2</div>
      <div id="ir3" class="ir">IR3</div>
      <div id="ir4" class="ir">IR4</div>
      <div id="ir5" class="ir">IR5</div>
    </div>
    <div id="landStatus" class="status safe" style="margin-top:14px;">✅ SAFE TO LAND</div>
  </div>

</div>

<script>
const ipEl       = document.getElementById('ip');
const fill       = document.getElementById('fill');
const tempRead   = document.getElementById('tempRead');
const vtol       = document.getElementById('vtol');
const speedRead  = document.getElementById('speedRead');
const moveState  = document.getElementById('moveState');
const landStatus = document.getElementById('landStatus');
const globBanner = document.getElementById('globBanner');

const irEls = [null,
  document.getElementById('ir1'),
  document.getElementById('ir2'),
  document.getElementById('ir3'),
  document.getElementById('ir4'),
  document.getElementById('ir5')
];

// Thapar University, Patiala (default / fallback location)
const THAPAR_LAT = 30.3523;
const THAPAR_LNG = 76.3646;

let map     = L.map('map').setView([THAPAR_LAT, THAPAR_LNG], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom:19 }).addTo(map);

// Default marker at Thapar (replaced with live GPS once fix arrives)
const fallbackIcon = L.icon({
  iconUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
  iconSize:[25,41], iconAnchor:[12,41], popupAnchor:[1,-34],
  shadowUrl:'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
  shadowSize:[41,41]
});
const liveIcon = L.icon({
  iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png',
  iconSize:[25,41], iconAnchor:[12,41], popupAnchor:[1,-34],
  shadowUrl:'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
  shadowSize:[41,41]
});
let marker    = L.marker([THAPAR_LAT, THAPAR_LNG], {icon: fallbackIcon})
                 .addTo(map)
                 .bindPopup('🏫 Thapar University (default)')
                 .openPopup();
let mapZoomed = false;  // becomes true once real GPS fix zooms the map

function clamp(n, a, b){ return Math.max(a, Math.min(b, n)); }

async function poll(){
  try {
    const r = await fetch('/data');
    const d = await r.json();

    // IP
    if (d.ip && ipEl.textContent.includes("—"))
      ipEl.textContent = "IP: " + d.ip;

    // GPS
    if (d.gpsValid) {
      marker.setLatLng([d.lat, d.lng]);
      marker.setIcon(liveIcon);
      marker.bindPopup('📍 Live GPS').openPopup();
      if (!mapZoomed) { map.setView([d.lat, d.lng], 17); mapZoomed = true; }
      else             { map.panTo([d.lat, d.lng]); }
    } else {
      marker.setLatLng([THAPAR_LAT, THAPAR_LNG]);
      marker.setIcon(fallbackIcon);
    }

    // Temperature  (-127 = DS18B20 not connected / wiring error)
    const t = d.tempC ?? -127;
    if (t <= -100) {
      fill.style.height = '0%';
      tempRead.textContent = 'Sensor Error';
      tempRead.style.fontSize = '22px';
      tempRead.style.color = '#ef4444';
    } else {
      fill.style.height = clamp((t / 60) * 100, 0, 100) + '%';
      tempRead.textContent = t.toFixed(1) + ' °C';
      tempRead.style.fontSize = '';
      tempRead.style.color = '';
    }

    // Speed / movement
    // Speed / movement
const sp = d.speed_mps ?? 0;
speedRead.textContent = "Speed: " + sp.toFixed(2) + " m/s";
moveState.textContent = d.move;

// tilt the drone using pitch & roll
const pitch = d.pitch_deg ?? 0;
const roll  = d.roll_deg  ?? 0;

// Move drone horizontally across the sky based on roll
const skyWidth  = document.getElementById('sky').offsetWidth;
const clampedRoll = Math.max(-45, Math.min(45, roll));
const xPos = ((clampedRoll + 45) / 90) * (skyWidth - 130); // 130 = vtol width

vtol.style.left      = xPos + "px";
vtol.style.transform = `rotate(${pitch}deg)`;  // nose up/down tilt

    // IR sensors
    const ir = d.ir ?? [1,1,1,1,1];
    let safe = true;
    for (let i = 0; i < 5; i++){
      const ok = (ir[i] === 1);
      irEls[i+1].classList.toggle('ok',  ok);
      irEls[i+1].classList.toggle('bad', !ok);
      if (!ok) safe = false;
    }
    landStatus.textContent = safe ? "✅ SAFE TO LAND" : "⚠ HAZARD";
    landStatus.className   = safe ? "status safe"     : "status danger";

    // Global banner
    const alert = d.alert ?? false;
    globBanner.textContent  = alert ? "🚨 ALERT"    : "✅ Normal";
    globBanner.className    = alert ? "banner alert" : "banner ok";

  } catch(e){
    console.log("Poll error:", e);
  }
  setTimeout(poll, 500);
}

poll();
</script>
</body>
</html>
)rawliteral";

// ================== Helper functions ==================
String movementStateFromAccel(float totalAccel){
  if (totalAccel < STILL_THRESH) return "Still";
  if (totalAccel < FAST_THRESH)  return "Normal movement";
  return "Fast movement";
}

bool alertFromSensors(bool blackDetected, float totalAccel){
  return (blackDetected || (totalAccel > FAST_THRESH));
}

// ================== Sensor read for /data endpoint ==================
void readSensorsForWeb(float &totalAccel_out, float &tempC_out, uint8_t (&ir)[5],
                       float &pitch_deg_out, float &roll_deg_out, float &rel_speed_out)
{
  // -- MPU6050 --
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(MPU_addr, (uint8_t)14, (uint8_t)true) != 14) return;

  AcX = (Wire.read()<<8) | Wire.read();
  AcY = (Wire.read()<<8) | Wire.read();
  AcZ = (Wire.read()<<8) | Wire.read();
  Wire.read(); Wire.read(); // skip internal temp
  GyX = (Wire.read()<<8) | Wire.read();
  GyY = (Wire.read()<<8) | Wire.read();
  GyZ = (Wire.read()<<8) | Wire.read();

  double ax = AcX, ay = AcY, az = AcZ;
  totalAccel_out = (float)sqrt(ax*ax + ay*ay + az*az);

  // -- IR --
  ir[0] = digitalRead(IR1);
  ir[1] = digitalRead(IR2);
  ir[2] = digitalRead(IR3);
  ir[3] = digitalRead(IR4);
  ir[4] = digitalRead(IR5);

  // -- Temperature --
  tempSensor.requestTemperatures();
  tempC_out = tempSensor.getTempCByIndex(0);

  // -- Time step --
  unsigned long now = millis();
  if (lastIMUms_forSpeed == 0) lastIMUms_forSpeed = now;
  double dt = (now - lastIMUms_forSpeed) / 1000.0;
  lastIMUms_forSpeed = now;
  if (dt <= 0) dt = 0.001;

  // -- Scalar speed (UI) --
  // KEY FIX: if device is still, kill speed immediately — don't let noise integrate
  if (totalAccel_out < STILL_THRESH) {
    speed_mps = 0.0;
  } else {
    double total_g   = totalAccel_out / LSB_PER_G;
    double lin_g_mag = total_g - g_baseline;
    if (fabs(lin_g_mag) < 0.08) lin_g_mag = 0;  // deadband: 0.03 -> 0.08
    double lin_ms2 = lin_g_mag * 9.81;
    speed_mps += lin_ms2 * dt;
    if (fabs(lin_ms2) < 0.30) speed_mps *= 0.85; // strong damping when coasting
    if (speed_mps < 0)    speed_mps = 0;
    if (speed_mps > 12.0) speed_mps = 12.0;
  }

  // -- Pitch & Roll --
  double roll_rad  = atan2(ay, az);
  double pitch_rad = atan2(-ax, sqrt(ay*ay + az*az));
  roll_deg_out  = (float)(roll_rad  * 57.2957795);
  pitch_deg_out = (float)(pitch_rad * 57.2957795);

  // -- Relative (horizontal) speed --
  // KEY FIX: zero out vector velocity when still
  if (totalAccel_out < STILL_THRESH) {
    vx_mps = 0.0;
    vy_mps = 0.0;
  } else {
    double ax_g = ax / LSB_PER_G;
    double ay_g = ay / LSB_PER_G;

    double cp = cos(pitch_rad), sp2 = sin(pitch_rad);
    double cr = cos(roll_rad),  sr  = sin(roll_rad);

    double gx_b = -sp2;
    double gy_b =  sr * cp;

    double linX_ms2 = (ax_g - gx_b) * 9.81;
    double linY_ms2 = (ay_g - gy_b) * 9.81;
    if (fabs(linX_ms2) < 0.20) linX_ms2 = 0;  // deadband: 0.08 -> 0.20
    if (fabs(linY_ms2) < 0.20) linY_ms2 = 0;

    vx_mps += linX_ms2 * dt;
    vy_mps += linY_ms2 * dt;
    vx_mps *= 0.97;  // stronger damping: 0.995 -> 0.97
    vy_mps *= 0.97;
    vx_mps = constrain(vx_mps, -20, 20);
    vy_mps = constrain(vy_mps, -20, 20);
  }

  rel_speed_out = (float)sqrt(vx_mps*vx_mps + vy_mps*vy_mps);
}

// ================== Web handlers ==================
void handleRoot(){
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData(){
  float tAccel = 0, tC = 0, pitchDeg = 0, rollDeg = 0, relSpeed = 0;
  uint8_t ir[5] = {1,1,1,1,1};

  readSensorsForWeb(tAccel, tC, ir, pitchDeg, rollDeg, relSpeed);

  bool blackDetected = (ir[0]==LOW || ir[1]==LOW || ir[2]==LOW || ir[3]==LOW || ir[4]==LOW);

  // Real GPS from NEO-6M
  uint8_t gpsSats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;

  String json = "{";
  json += "\"ip\":\""          + WiFi.localIP().toString()   + "\",";
  json += "\"tempC\":"         + String(tC)                  + ",";
  json += "\"speed_mps\":"     + String(speed_mps)           + ",";
  json += "\"gpsValid\":"      + String(gps_valid ? "true" : "false") + ",";
  json += "\"gpsSats\":"       + String(gpsSats)             + ",";
  if (gps_valid) {
    json += "\"lat\":"  + String(gps_lat, 6) + ",";
    json += "\"lng\":"  + String(gps_lng, 6) + ",";
  } else {
    // Fallback: Thapar University, Patiala
    json += "\"lat\":30.352300,\"lng\":76.364600,";
  }
  json += "\"totalAccel\":"    + String(tAccel, 0)                            + ",";
  json += "\"totalAccel_g\":"  + String(tAccel / LSB_PER_G, 3)               + ",";
  json += "\"rel_speed_mps\":" + String(relSpeed, 3)                          + ",";
  json += "\"pitch_deg\":"     + String(pitchDeg, 1)                          + ",";
  json += "\"roll_deg\":"      + String(rollDeg, 1)                           + ",";
  json += "\"move\":\""        + movementStateFromAccel(tAccel)               + "\",";
  json += "\"ir\":["
        + String(ir[0]) + "," + String(ir[1]) + "," + String(ir[2]) + ","
        + String(ir[3]) + "," + String(ir[4]) + "],";
  json += "\"alert\":"         + String(alertFromSensors(blackDetected, tAccel) ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

// ================== Gravity calibration ==================
void calibrateGravityUI(){
  const int N = 60;
  double sum_g = 0;
  for (int i = 0; i < N; i++){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) continue;
    if (Wire.requestFrom(MPU_addr, (uint8_t)6, (uint8_t)true) != 6) continue;
    int16_t ax = (Wire.read()<<8)|Wire.read();
    int16_t ay = (Wire.read()<<8)|Wire.read();
    int16_t az = (Wire.read()<<8)|Wire.read();
    double mag = sqrt((double)ax*ax + (double)ay*ay + (double)az*az);
    sum_g += (mag / LSB_PER_G);
    delay(8);
  }
  g_baseline = (float)(sum_g / N);
  if (g_baseline < 0.8f || g_baseline > 1.2f) g_baseline = 1.0f;
}

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);

  // NEO-6M GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART2 on GPIO16(RX)/GPIO17(TX) @ 9600 baud");

  Wire.begin(19, 18); // SDA=19, SCL=18

  // Wake MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("MPU6050 Initialized.");

  // IR inputs
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  // Outputs
  pinMode(BUZZER,    OUTPUT);
  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED,  OUTPUT);
  digitalWrite(BUZZER,    LOW);
  digitalWrite(RED_LED,   LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED,  LOW);

  tempSensor.begin();
  Serial.println("System Ready.\n");

  // WiFi
  Serial.print("WiFi: connecting to "); Serial.println(SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30){ delay(300); Serial.print("."); tries++; }

  if (WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed. Starting AP 'ESP32-Dashboard'...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-Dashboard");
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  }

  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.begin();

  Serial.println("Calibrating gravity... keep device still.");
  calibrateGravityUI();
  Serial.print("g_baseline = "); Serial.println(g_baseline, 3);
}

// ================== LOOP ==================
void loop(){
  server.handleClient();

  // Feed GPS – must be called every loop iteration
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid() && gps.location.age() < 2000) {
        gps_lat   = (float)gps.location.lat();
        gps_lng   = (float)gps.location.lng();
        gps_valid = true;
      }
    }
  }
  if (gps_valid && gps.location.age() > 5000) gps_valid = false;  // stale fix

  // Read MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  AcX = (Wire.read()<<8) | Wire.read();
  AcY = (Wire.read()<<8) | Wire.read();
  AcZ = (Wire.read()<<8) | Wire.read();
  Wire.read(); Wire.read();
  GyX = (Wire.read()<<8) | Wire.read();
  GyY = (Wire.read()<<8) | Wire.read();
  GyZ = (Wire.read()<<8) | Wire.read();

  float totalAccel = (float)sqrt((double)AcX*AcX + (double)AcY*AcY + (double)AcZ*AcZ);
  String moveStatus = movementStateFromAccel(totalAccel);

  // Read IR
  int v1 = digitalRead(IR1);
  int v2 = digitalRead(IR2);
  int v3 = digitalRead(IR3);
  int v4 = digitalRead(IR4);
  int v5 = digitalRead(IR5);
  bool blackDetected = (v1==LOW || v2==LOW || v3==LOW || v4==LOW || v5==LOW);

  // Read temp
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  // Alert logic
  bool alert = false;
  String reason = "";
  if (blackDetected)           { alert = true; reason += "[IR black] "; }
  if (totalAccel > FAST_THRESH){ alert = true; reason += "[Fast movement] "; }

  // Actuators
  if (alert){
    digitalWrite(BUZZER,    HIGH);
    digitalWrite(RED_LED,   HIGH);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED,  LOW);
  } else {
    digitalWrite(BUZZER,    LOW);
    digitalWrite(RED_LED,   LOW);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BLUE_LED,  LOW);
  }

  // Serial debug
  Serial.printf("Accel: %d, %d, %d | total=%.0f -> %s\n",
                AcX, AcY, AcZ, totalAccel, moveStatus.c_str());
  Serial.printf("IR: %d %d %d %d %d | Temp: %.2f C | %s\n",
                v1, v2, v3, v4, v5, tempC,
                alert ? ("ALERT " + reason).c_str() : "All normal.");

  delay(500);
}
