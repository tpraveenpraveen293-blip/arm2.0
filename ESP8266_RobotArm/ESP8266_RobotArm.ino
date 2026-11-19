/*
  ====================================================================================
  4-DOF HIGH-TORQUE ROBOTIC ARM CONTROLLER FOR ESP8266
  ====================================================================================
  Hardware Setup:
    - 2x 150 kg.cm High-Torque Servos (Base/Shoulder, Elbow)
    - 2x MG996R Servos (Wrist Roll/Yaw, Gripper)
    - ESP8266 (NodeMCU / Wemos D1 Mini / ESP-12)
    - External 6.0V - 7.4V High-Current Power Supply (10A - 20A rated)

  Communication Modes:
    1. WebSocket over Wi-Fi (Port 81) - Real-time low-latency bidirectional
  control
    2. HTTP Web Server (Port 80) - Status telemetry & REST API
    3. Serial (115200 baud) - Direct USB cable control via browser Web Serial
  API

  SAFETY FEATURE: Smooth S-Curve / Velocity-limited trajectory interpolation to
  prevent massive inertial torque shock on the 150kg servos and long tubular
  links.
  ====================================================================================
*/

#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>

// ====================================================================================
// HARDWARE CONFIGURATION
// ====================================================================================
// Set to true if you are using an I2C PCA9685 16-Channel PWM Servo Driver
// module. Set to false if you are connecting servo signal pins directly to
// ESP8266 GPIOs.
#define USE_PCA9685 false

#if USE_PCA9685
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 150 // Minimum pulse length count (out of 4096) for 0 deg
#define SERVOMAX 600 // Maximum pulse length count (out of 4096) for 180 deg
#else
#include <Servo.h>
Servo servos[4];
// Explicit ESP8266 GPIO definitions to ensure 100% board compatibility
#ifndef D1
#define D1 5  // GPIO 5 (Gripper Jaw)
#endif
#ifndef D2
#define D2 4  // GPIO 4 (Wrist Roll / Yaw)
#endif
#ifndef D4
#define D4 2  // GPIO 2 (Elbow Joint)
#endif
#ifndef D5
#define D5 14 // GPIO 14 (Base Joint)
#endif

// Safe ESP8266 GPIO pin mapping:
// D5 (GPIO 14) -> Servo 1: Base / Shoulder Joint (150kg)
// D4 (GPIO 2)  -> Servo 2: Elbow Joint (150kg)
// D2 (GPIO 4)  -> Servo 3: Wrist Roll / Yaw Joint (MG996R)
// D1 (GPIO 5)  -> Servo 4: Gripper Jaw (MG996R)
const int SERVO_PINS[4] = {D5, D4, D2, D1};
#endif

// ====================================================================================
// WI-FI CREDENTIALS
// ====================================================================================
// Enter your local home/office Wi-Fi details.
// If the ESP8266 fails to connect within 10 seconds, it will automatically
// broadcast its own Access Point hotspot: "RoboArm-ESP8266" (Password:
// "12345678").
const char *WIFI_SSID = "Praveen";
const char *WIFI_PASSWORD = "123456789";

const char *AP_SSID = "RoboArm-ESP8266";
const char *AP_PASSWORD = "12345678";

// ====================================================================================
// SERVO CALIBRATION & SAFETY LIMITS
// ====================================================================================
const int NUM_SERVOS = 4;

// Physical travel limits for each joint to prevent self-collision
const float MIN_ANGLES[NUM_SERVOS] = {0.0, 0.0, 0.0, 0.0};
const float MAX_ANGLES[NUM_SERVOS] = {180.0, 180.0, 180.0, 180.0}; // Full 0-180 range for all joints including Gripper (D1)

// Home / Default Startup Position (Arm safe folded/balanced stance)
const float HOME_ANGLES[NUM_SERVOS] = {90.0, 90.0, 90.0, 20.0};

// Current dynamic positions and target setpoints (in floating-point degrees)
float currentAngles[NUM_SERVOS] = {90.0, 90.0, 90.0, 20.0};
float targetAngles[NUM_SERVOS] = {90.0, 90.0, 90.0, 20.0};

// S-Curve motion tracking per servo
float startAngles[NUM_SERVOS] = {90.0, 90.0, 90.0, 20.0};
float prevTargets[NUM_SERVOS] = {90.0, 90.0, 90.0, 20.0};
unsigned long motionStartMillis[NUM_SERVOS] = {0, 0, 0, 0};

// Motion speed in degrees per second (default: 90 deg/s for fast, responsive motion)
float globalSpeedDegPerSec = 90.0;
unsigned long lastTelemetryMillis = 0;

// Servers
ESP8266WebServer server(80);
WebSocketsServer webSocket(81);

// Serial input buffer for Web Serial API
String serialBuffer = "";

// ====================================================================================
// HELPER FUNCTIONS: SERVO WRITING & HARDWARE MAPPING
// ====================================================================================
void applyServoAngle(int index, float angleDeg) {
  // Clamp angle to safe limits
  if (angleDeg < MIN_ANGLES[index])
    angleDeg = MIN_ANGLES[index];
  if (angleDeg > MAX_ANGLES[index])
    angleDeg = MAX_ANGLES[index];

#if USE_PCA9685
  // Map 0 - 180 degrees to PCA9685 12-bit pulse ticks (approx 150 to 600 at
  // 50Hz)
  int pulse = map((int)angleDeg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(index, 0, pulse);
#else
  int pin = SERVO_PINS[index];
  if (!servos[index].attached()) {
    servos[index].attach(pin);
  }
  servos[index].write((int)round(angleDeg));
#endif
}

// ====================================================================================
// S-CURVE EASING FUNCTION (SMOOTHSTEP: 3t^2 - 2t^3)
// ====================================================================================
// Fast acceleration into peak cruising speed with smooth ease-in & ease-out
float sCurveEase(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

// ====================================================================================
// MOTION INTERPOLATION ENGINE (S-CURVE: SLOW → FAST → SLOW)
// ====================================================================================
void updateMotionInterpolation() {
  unsigned long nowMillis = millis();

  for (int i = 0; i < NUM_SERVOS; i++) {
    // Detect when target changes → start a new S-curve motion
    if (abs(targetAngles[i] - prevTargets[i]) > 0.1f) {
      startAngles[i] = currentAngles[i];
      prevTargets[i] = targetAngles[i];
      motionStartMillis[i] = nowMillis;
    }

    float totalDist = abs(targetAngles[i] - startAngles[i]);
    if (totalDist < 0.05f) {
      currentAngles[i] = targetAngles[i];
      continue;
    }

    // Duration of the full move based on distance and speed setting
    float durationMs = (totalDist / globalSpeedDegPerSec) * 1000.0f;
    float elapsed = (float)(nowMillis - motionStartMillis[i]);
    float progress = elapsed / durationMs;

    if (progress >= 1.0f) {
      currentAngles[i] = targetAngles[i];
    } else {
      // Apply S-curve easing: slow → fast → slow
      float eased = sCurveEase(progress);
      currentAngles[i] =
          startAngles[i] + (targetAngles[i] - startAngles[i]) * eased;
    }

    applyServoAngle(i, currentAngles[i]);
  }
}

// ====================================================================================
// JSON COMMAND PARSING
// ====================================================================================
void processJsonCommand(const char *payload) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print(F("[JSON] Parse error: "));
    Serial.println(error.c_str());
    return;
  }

  const char *cmd = doc["cmd"] | "";

  // 1. Set all target angles:
  // {"cmd":"set_angles","angles":[90,110,45,30],"speed":90}
  if (strcmp(cmd, "set_angles") == 0) {
    JsonArray arr = doc["angles"];
    int count = (arr.size() < NUM_SERVOS) ? arr.size() : NUM_SERVOS;
    for (int i = 0; i < count; i++) {
      float val = arr[i].as<float>();
      targetAngles[i] = constrain(val, MIN_ANGLES[i], MAX_ANGLES[i]);
    }
    Serial.printf("[CMD] WS Angles: J1=%.0f, J2=%.0f, J3=%.0f, J4(D1)=%.0f\n",
                  targetAngles[0], targetAngles[1], targetAngles[2], targetAngles[3]);
    if (doc.containsKey("speed")) {
      float spd = doc["speed"].as<float>();
      if (spd >= 5.0f && spd <= 300.0f) {
        globalSpeedDegPerSec = spd;
      }
    }
  }
  // 2. Set individual servo: {"cmd":"set_servo","index":3,"angle":90}
  else if (strcmp(cmd, "set_servo") == 0) {
    int idx = doc["index"] | -1;
    float ang = doc["angle"] | -1.0f;
    if (idx >= 0 && idx < NUM_SERVOS) {
      targetAngles[idx] = constrain(ang, MIN_ANGLES[idx], MAX_ANGLES[idx]);
    }
  }
  // 3. Set speed: {"cmd":"set_speed","speed":90}
  else if (strcmp(cmd, "set_speed") == 0) {
    float spd = doc["speed"] | 90.0f;
    if (spd >= 5.0f && spd <= 300.0f) {
      globalSpeedDegPerSec = spd;
    }
  }
  // 4. Emergency Stop / Immediate Hold: {"cmd":"stop"}
  else if (strcmp(cmd, "stop") == 0) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      targetAngles[i] = currentAngles[i];
    }
  }
  // 5. Home: {"cmd":"home"}
  else if (strcmp(cmd, "home") == 0) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      targetAngles[i] = HOME_ANGLES[i];
    }
  }
}

// Compact line parser: "S:90,110,45,90,30" or "S1:90"
void processRawStringCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0)
    return;

  if (cmd.startsWith("S:")) {
    // Format: S:90,110,45,30
    String data = cmd.substring(2);
    int idx = 0;
    while (data.length() > 0 && idx < NUM_SERVOS) {
      int comma = data.indexOf(',');
      String valStr = (comma >= 0) ? data.substring(0, comma) : data;
      float val = valStr.toFloat();
      targetAngles[idx] = constrain(val, MIN_ANGLES[idx], MAX_ANGLES[idx]);
      if (comma >= 0) {
        data = data.substring(comma + 1);
      } else {
        break;
      }
      idx++;
    }
    Serial.printf("[CMD] Serial Angles: J1=%.0f, J2=%.0f, J3=%.0f, J4(D1)=%.0f\n",
                  targetAngles[0], targetAngles[1], targetAngles[2], targetAngles[3]);
  } else if (cmd.startsWith("{")) {
    processJsonCommand(cmd.c_str());
  } else if (cmd.equalsIgnoreCase("HOME")) {
    for (int i = 0; i < NUM_SERVOS; i++)
      targetAngles[i] = HOME_ANGLES[i];
  } else if (cmd.equalsIgnoreCase("STOP")) {
    for (int i = 0; i < NUM_SERVOS; i++)
      targetAngles[i] = currentAngles[i];
  }
}

// ====================================================================================
// WEBSOCKET EVENTS
// ====================================================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    Serial.printf("[WS] Client #%u disconnected\n", num);
    break;

  case WStype_CONNECTED: {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[WS] Client #%u connected from %d.%d.%d.%d\n", num, ip[0],
                  ip[1], ip[2], ip[3]);

    // Immediately send greeting & current angles to new client
    StaticJsonDocument<256> doc;
    doc["type"] = "welcome";
    JsonArray cur = doc.createNestedArray("current");
    JsonArray tgt = doc.createNestedArray("target");
    for (int i = 0; i < NUM_SERVOS; i++) {
      cur.add(currentAngles[i]);
      tgt.add(targetAngles[i]);
    }
    doc["speed"] = globalSpeedDegPerSec;
    String out;
    serializeJson(doc, out);
    webSocket.sendTXT(num, out);
    break;
  }

  case WStype_TEXT: {
    String msg = String((char *)payload);
    if (msg.startsWith("{")) {
      processJsonCommand((char *)payload);
    } else {
      processRawStringCommand(msg);
    }
    break;
  }

  default:
    break;
  }
}

// ====================================================================================
// HTTP SERVER HANDLERS
// ====================================================================================
void handleRoot() {
  String html =
      F("<!DOCTYPE html><html><head><title>ESP8266 Robotic Arm</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>body{background:#111827;color:#f3f4f6;font-family:sans-serif;"
        "padding:2rem;text-align:center;}"
        ".card{background:#1f2937;border:1px solid "
        "#374151;border-radius:12px;padding:20px;max-width:500px;margin:auto;}"
        "h1{color:#38bdf8;margin-bottom:8px;}p{color:#9ca3af;font-size:14px;}"
        ".badge{display:inline-block;padding:6px "
        "12px;background:#059669;color:#fff;border-radius:999px;font-size:12px;"
        "font-weight:bold;margin:10px 0;}"
        "a{color:#38bdf8;text-decoration:none;font-weight:bold;}"
        "</style></head><body>"
        "<div class='card'>"
        "<h1>ESP8266 Robotic Arm Core</h1>"
        "<div class='badge'>WebSocket &amp; Serial Active</div>"
        "<p>WebSocket Server listening on port <b>81</b></p>"
        "<p>Launch the 3D Live Simulation on your PC browser (Localhost) to "
        "control the arm with real-time inverse kinematics!</p>"
        "</div></body></html>");
  server.send(200, "text/html", html);
}

void handleStatusApi() {
  StaticJsonDocument<256> doc;
  doc["status"] = "online";
  doc["uptime"] = millis() / 1000;
  doc["speed"] = globalSpeedDegPerSec;
  JsonArray cur = doc.createNestedArray("current");
  for (int i = 0; i < NUM_SERVOS; i++)
    cur.add(currentAngles[i]);
  String response;
  serializeJson(doc, response);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
}

// ====================================================================================
// SETUP
// ====================================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("    4-DOF HIGH-TORQUE ROBOTIC ARM CONTROLLER     "));
  Serial.println(F("=================================================="));

  // Initialize Servos
#if USE_PCA9685
  Serial.println(
      F("[SERVO] Initializing PCA9685 I2C driver on SDA(D2), SCL(D1)..."));
  Wire.begin(D2, D1);
  pwm.begin();
  pwm.setPWMFreq(50); // Standard analog/digital servo 50Hz update rate
#else
  Serial.println(F("[SERVO] Initializing Direct ESP8266 GPIO pins..."));
  for (int i = 0; i < NUM_SERVOS; i++) {
    pinMode(SERVO_PINS[i], OUTPUT);
    servos[i].attach(SERVO_PINS[i]);
    Serial.printf("[SERVO] Joint %d attached to Pin %d (Status: %s)\n",
                  i + 1, SERVO_PINS[i], servos[i].attached() ? "READY" : "FAILED");
  }

  // Startup test pulse on Joint 4 (D1 Gripper) & Joint 3 (D2 Wrist Roll) to confirm wiring & power
  delay(100);
  servos[3].write(45);
  servos[2].write(110);
  delay(250);
  servos[3].write(20);
  servos[2].write(90);
#endif

  // Move smoothly to safe startup home position
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = HOME_ANGLES[i];
    targetAngles[i] = HOME_ANGLES[i];
    applyServoAngle(i, HOME_ANGLES[i]);
  }
  Serial.println(F("[SERVO] All joints set to safe home stance."));

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  if (strlen(WIFI_SSID) > 0 && strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0) {
    Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500);
      Serial.print(".");
      retries++;
    }
    Serial.println();
  }

  // If WiFi failed to connect or not configured, start SoftAP hotspot
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WIFI] Connected! IP Address: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(
        F("[WIFI] Could not connect to router. Starting Access Point mode..."));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print(F("[WIFI] Access Point active. SSID: "));
    Serial.print(AP_SSID);
    Serial.print(F(" | AP IP Address: "));
    Serial.println(WiFi.softAPIP());
  }

  // Start WebSocket Server on Port 81
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println(F("[WS] WebSocket Server started on port 81"));

  // Start HTTP Web Server on Port 80
  server.on("/", handleRoot);
  server.on("/api/status", handleStatusApi);
  server.begin();
  Serial.println(F("[HTTP] Web Server started on port 80"));
  Serial.println(
      F("[SYSTEM] Ready for incoming WebSocket and Serial commands!"));
}

// ====================================================================================
// MAIN LOOP
// ====================================================================================
void loop() {
  // Service background network tasks
  webSocket.loop();
  server.handleClient();

  // Read USB Web Serial incoming characters
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processRawStringCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      if (serialBuffer.length() < 256) {
        serialBuffer += c;
      }
    }
  }

  // Run the smooth acceleration/velocity-limited servo movement engine
  updateMotionInterpolation();

  // Periodic Telemetry broadcast (10Hz / 100ms interval) to all connected
  // clients
  unsigned long now = millis();
  if (now - lastTelemetryMillis >= 100) {
    lastTelemetryMillis = now;
    if (webSocket.connectedClients() > 0) {
      StaticJsonDocument<256> doc;
      doc["type"] = "telemetry";
      JsonArray cur = doc.createNestedArray("current");
      for (int i = 0; i < NUM_SERVOS; i++) {
        cur.add(round(currentAngles[i] * 10.0f) / 10.0f);
      }
      String out;
      serializeJson(doc, out);
      webSocket.broadcastTXT(out);
    }
  }
}
