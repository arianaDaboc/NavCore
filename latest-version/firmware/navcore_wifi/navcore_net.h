// Wi-Fi, web server (runs on core 0) and the telemetry shared with the control loop.
#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <stdarg.h>
#include <Update.h>
#include "app_gz.h"

#include "wifi_secrets.h"
#ifndef OTA_PASS
#define OTA_PASS AP_PASS
#endif
#define HOSTNAME  "navcore"

extern float headingTotal;
extern float pathHeading;
extern float ROUTE_LENGTH_CM;
extern float OBSTACLE_MAX_DEPTH_CM;
extern float PULSES_PER_CM;
extern float GYRO_SCALE;
void saveCalibration(float ppcm, float gs);
void resetCalibration();

volatile float gX = 0, gY = 0;
const char *volatile gMode = "IDLE";
volatile int gAvoid = 0;
volatile int gFront = -1, gLeft = -1, gRight = -1;
volatile unsigned long gFrontT = 0, gLeftT = 0, gRightT = 0;
volatile float gOx = -999, gOy = -999;
volatile int gSide = 0;
volatile float gEdgeY = -999, gGoalY = -999, gRearX = -999;
volatile float gFaceX = -999;
volatile bool gStartReq = false, gAbortReq = false, gRunning = false;
volatile bool gTurnTestReq = false;
volatile float gTurnTestDeg = -1;

#define EV_N 16
#define EV_LEN 72
static char evBuf[EV_N][EV_LEN];
static unsigned long evSeq = 0;
static portMUX_TYPE evMux = portMUX_INITIALIZER_UNLOCKED;

void netEvent(const char *fmt, ...) {
  char tmp[EV_LEN];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  for (char *p = tmp; *p; p++) if (*p == '"' || *p == '\\') *p = ' ';
  portENTER_CRITICAL(&evMux);
  strncpy(evBuf[evSeq % EV_N], tmp, EV_LEN - 1);
  evBuf[evSeq % EV_N][EV_LEN - 1] = 0;
  evSeq++;
  portEXIT_CRITICAL(&evMux);
}

#define POSE_N 512
#define POSE_MAX_PER_REPLY 80
static float poseBuf[POSE_N][2];
static unsigned long poseSeq = 0;
static float poseLastX = 0, poseLastY = 0;

void netPose(float x, float y, bool force = false) {
  float dx = x - poseLastX, dy = y - poseLastY;
  if (!force && dx * dx + dy * dy < 1.0f * 1.0f) return;
  poseLastX = x; poseLastY = y;
  portENTER_CRITICAL(&evMux);
  poseBuf[poseSeq % POSE_N][0] = x;
  poseBuf[poseSeq % POSE_N][1] = y;
  poseSeq++;
  portEXIT_CRITICAL(&evMux);
}

static WebServer server(80);

static void sendCors() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

static void handleRoot() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", (PGM_P)APP_GZ, APP_GZ_LEN);
}

static int ageOrMinus(int v, unsigned long t) {
  return (millis() - t > 1500) ? -1 : v;
}

// Robot state as JSON, plus only the events and trail points the app has not received yet.
static void handleState() {
  static char buf[3200];
  unsigned long since = server.hasArg("since") ? strtoul(server.arg("since").c_str(), NULL, 10) : 0;
  unsigned long psince = server.hasArg("ps") ? strtoul(server.arg("ps").c_str(), NULL, 10) : 0;
  unsigned long seq;
  int n = 0;

  n += snprintf(buf + n, sizeof(buf) - n,
                "{\"v\":17,\"t\":%lu,\"run\":%d,\"m\":\"%s\",\"x\":%.1f,\"y\":%.1f,\"th\":%.1f,"
                "\"rt\":%.0f,\"dp\":%.0f,\"av\":%d,\"f\":%d,\"l\":%d,\"r\":%d,"
                "\"ox\":%.1f,\"oy\":%.1f,\"sd\":%d,\"ey\":%.1f,\"gy\":%.1f,\"bx\":%.1f,\"fx\":%.1f,\"rs\":%d,"
                "\"cp\":%.4f,\"cg\":%.4f,\"tt\":%.1f,",
                millis(), gRunning ? 1 : 0, (const char *)gMode, (float)gX, (float)gY,
                (float)(headingTotal - pathHeading), ROUTE_LENGTH_CM, OBSTACLE_MAX_DEPTH_CM, (int)gAvoid,
                ageOrMinus(gFront, gFrontT), ageOrMinus(gLeft, gLeftT), ageOrMinus(gRight, gRightT),
                (float)gOx, (float)gOy, (int)gSide, (float)gEdgeY, (float)gGoalY, (float)gRearX, (float)gFaceX,
                (int)WiFi.RSSI(), PULSES_PER_CM, GYRO_SCALE, (float)gTurnTestDeg);

  portENTER_CRITICAL(&evMux);
  seq = evSeq;
  n += snprintf(buf + n, sizeof(buf) - n, "\"es\":%lu,\"ev\":[", seq);
  unsigned long first = (seq > EV_N) ? seq - EV_N : 0;
  if (since > first) first = since;
  bool comma = false;
  for (unsigned long i = first; i < seq && n < (int)sizeof(buf) - EV_LEN - 16; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, "%s[%lu,\"%s\"]", comma ? "," : "", i + 1, evBuf[i % EV_N]);
    comma = true;
  }
  n += snprintf(buf + n, sizeof(buf) - n, "]");

  unsigned long pfirst = (poseSeq > POSE_N) ? poseSeq - POSE_N : 0;
  if (psince > pfirst) pfirst = psince;
  unsigned long plast = pfirst + POSE_MAX_PER_REPLY;
  if (plast > poseSeq) plast = poseSeq;
  static float pts[POSE_MAX_PER_REPLY][2];
  int np = 0;
  for (unsigned long i = pfirst; i < plast; i++, np++) {
    pts[np][0] = poseBuf[i % POSE_N][0];
    pts[np][1] = poseBuf[i % POSE_N][1];
  }
  portEXIT_CRITICAL(&evMux);
  n += snprintf(buf + n, sizeof(buf) - n, ",\"ps\":%lu,\"p\":[", plast);
  for (int i = 0; i < np && n < (int)sizeof(buf) - 40; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, "%s[%.1f,%.1f]", i ? "," : "", pts[i][0], pts[i][1]);
  }
  n += snprintf(buf + n, sizeof(buf) - n, "]}");

  sendCors();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

// Commands from the app: start, stop, reset, turn test, calibration.
static void handleCmd() {
  String c = server.arg("c");
  sendCors();
  if (c == "start") {
    if (gRunning) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"already running\"}"); return; }
    if (server.hasArg("route")) {
      float r = server.arg("route").toFloat();
      if (r >= 100 && r <= 600) ROUTE_LENGTH_CM = r;
    }
    if (server.hasArg("depth")) {
      float dv = server.arg("depth").toFloat();
      if (dv >= 5 && dv <= 80) OBSTACLE_MAX_DEPTH_CM = dv;
    }
    gStartReq = true;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"started\"}");
  } else if (c == "stop") {
    gAbortReq = true;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"stopped\"}");
  } else if (c == "reset") {
    if (gRunning) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"the robot is still running\"}"); return; }

    gX = 0; gY = 0;
    pathHeading = headingTotal;
    netPose(0, 0, true);
    gOx = gOy = gEdgeY = gGoalY = gRearX = gFaceX = -999;
    gSide = 0; gAvoid = 0;
    gMode = "IDLE";
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"reset\"}");
  } else if (c == "turn") {
    if (gRunning) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"the robot is still running\"}"); return; }
    gTurnTestReq = true;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"turn test\"}");
  } else if (c == "calib") {
    if (gRunning) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"the robot is still running\"}"); return; }
    float ppcm = server.hasArg("ppcm") ? server.arg("ppcm").toFloat() : PULSES_PER_CM;
    float gs = server.hasArg("gs") ? server.arg("gs").toFloat() : GYRO_SCALE;

    if (ppcm < 1.5f || ppcm > 3.0f || gs < 0.85f || gs > 1.15f) {
      server.send(200, "application/json", "{\"ok\":false,\"msg\":\"value out of range\"}");
      return;
    }
    saveCalibration(ppcm, gs);
    netEvent("Calibration saved: %.4f pulses/cm, gyro scale %.4f", ppcm, gs);
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"calibration saved\"}");
  } else if (c == "calibreset") {
    if (gRunning) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"the robot is still running\"}"); return; }
    resetCalibration();
    netEvent("Calibration: default values restored");
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"defaults restored\"}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"unknown command\"}");
  }
}

static bool otaOk = false;
static String otaMsg;

static void handleUpdateUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    otaOk = false;
    if (gRunning) { otaMsg = "the robot is running - stop it first"; return; }
    if (server.arg("key") != OTA_PASS) { otaMsg = "wrong password"; return; }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { otaMsg = "could not start the update"; return; }
    otaOk = true;
    otaMsg = "";
    netEvent("Firmware update: receiving %s", up.filename.c_str());
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (otaOk && Update.write(up.buf, up.currentSize) != up.currentSize) {
      otaOk = false;
      otaMsg = "error writing to flash";
      Update.abort();
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (otaOk && !Update.end(true)) {
      otaOk = false;
      otaMsg = "invalid file (not an ESP32 firmware?)";
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (otaOk) Update.abort();
    otaOk = false;
    otaMsg = "transfer interrupted";
  }
}

static void handleUpdateDone() {
  sendCors();
  if (otaOk) {
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"new firmware uploaded, the robot is restarting\"}");
    netEvent("New firmware uploaded, restarting");
    delay(800);
    ESP.restart();
  } else {
    String body = String("{\"ok\":false,\"msg\":\"") + (otaMsg.length() ? otaMsg : String("update failed")) + "\"}";
    server.send(200, "application/json", body);
  }
}

static void handleNotFound() { server.send(404, "text/plain", "Not found"); }

static void netTask(void *) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void netSetup() {
  WiFi.persistent(false);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  bool sta = false;

  if (strcmp(WIFI_SSID, "network_name") != 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to Wi-Fi ");
    Serial.print(WIFI_SSID);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    sta = (WiFi.status() == WL_CONNECTED);
    if (!sta) {
      Serial.print("Connection failed. Wi-Fi code: ");
      Serial.println((int)WiFi.status());
      Serial.println("On iPhone: enable Allow Others to Join and Maximise Compatibility.");
    }
  }

  if (!sta) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("Own network: ");
    Serial.print(AP_SSID);
    Serial.print(" (password ");
    Serial.print(AP_PASS);
    Serial.println(")");
    Serial.print("Open: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("Connected. Open: http://");
    Serial.print(WiFi.localIP());
    Serial.print("  or  http://");
    Serial.print(HOSTNAME);
    Serial.println(".local");
  }

  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);

  server.on("/", handleRoot);
  server.on("/api/state", handleState);
  server.on("/api/cmd", handleCmd);
  server.on("/api/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.onNotFound(handleNotFound);
  server.begin();

  xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 1, NULL, 0);
}
