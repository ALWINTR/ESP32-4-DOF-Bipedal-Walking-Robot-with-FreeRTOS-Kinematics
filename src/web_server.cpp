#include "web_server.h"
#include "web_page.h"
#include "config.h"
#include "pca_driver.h"
#include "kinematics.h"

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>

static WebServer server(80);
static WiFiMulti wifiMulti;

static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleGetAngles() {
  LocomotionTelemetry telem = getLocomotionTelemetry();
  String json = "{\"angles\":{";
  for (int i = 0; i < NUM_SERVOS; i++) {
    JointState* j = getJoint(i);
    if (j) {
      json += "\"" + String(j->channel) + "\":" + String((int)round(j->currentAngle));
      if (i < NUM_SERVOS - 1) json += ",";
    }
  }
  json += "},\"active_gait\":\"" + telem.activeGait + "\"";
  json += ",\"gait_running\":" + String(telem.isRunning ? "true" : "false");
  json += ",\"pca_found\":" + String(isPCAFound() ? "true" : "false");
  json += ",\"weight_l\":" + String(telem.weightLeftGrams, 1);
  json += ",\"weight_r\":" + String(telem.weightRightGrams, 1);
  json += ",\"force_n\":" + String(telem.groundForceNewtons, 2);
  json += ",\"contact\":\"" + telem.contactState + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

static void handleRunGait() {
  if (!server.hasArg("gait")) {
    server.send(400, "application/json", "{\"error\":\"Missing gait parameter\"}");
    return;
  }
  String gait = server.arg("gait");
  if (gait == "stand" || gait == "stop") {
    stopLocomotion();
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"stand\"}");
  } else if (gait == "start" || gait == "walk" || gait == "forward") {
    startLocomotion("walk");
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"walk\"}");
  } else if (gait == "step") {
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"step\"}");
    executeSingleStep();
  } else if (gait == "test" || gait == "test_joints") {
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"test\"}");
    runJointDiagnostics();
  } else {
    startLocomotion(gait);
    server.send(200, "application/json", "{\"status\":\"ok\",\"gait\":\"" + gait + "\"}");
  }
}

static void handleSetServo() {
  if (!server.hasArg("ch") || !server.hasArg("val")) {
    server.send(400, "application/json", "{\"error\":\"Missing arguments ch or val\"}");
    return;
  }
  int ch = server.arg("ch").toInt();
  float val = server.arg("val").toFloat();

  stopLocomotion(); // Switch to manual control
  setServoAngle(ch, val);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

static void handleSetGaitParam() {
  if (!server.hasArg("param") || !server.hasArg("val")) {
    server.send(400, "application/json", "{\"error\":\"Missing arguments\"}");
    return;
  }
  String param = server.arg("param");
  float val = server.arg("val").toFloat();
  setGaitParam(param, val);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

static void handleSaveOffsets() {
  saveNVSJointOffsets();
  server.send(200, "application/json", "{\"status\":\"saved\"}");
}

static void handleResetOffsets() {
  resetNVSJointOffsets();
  server.send(200, "application/json", "{\"status\":\"reset\"}");
}

static void handleScanI2C() {
  String json = "{\"devices\":[";
  int count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (count > 0) json += ",";
      char hexBuf[10];
      snprintf(hexBuf, sizeof(hexBuf), "\"0x%02X\"", addr);
      json += hexBuf;
      count++;
    }
  }
  json += "],\"pca_found\":" + String(isPCAFound() ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void initWebServer() {
  // 1. Enable Dual Mode (Station + SoftAP)
  WiFi.mode(WIFI_AP_STA);

  // 2. Always launch SoftAP with guaranteed static IP 192.168.4.1
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println("\n[WiFi] SoftAP Started!");
  Serial.printf("  SSID: %s | PASS: %s\n", AP_SSID, AP_PASS);
  Serial.printf("  Direct Access Point URL: http://%s\n", WiFi.softAPIP().toString().c_str());

  // 3. Register Station Wi-Fi Networks in priority order
  for (size_t i = 0; i < NUM_NETWORKS; i++) {
    wifiMulti.addAP(KNOWN_NETWORKS[i].ssid, KNOWN_NETWORKS[i].password);
    Serial.printf("[WiFi] Registered network: %s\n", KNOWN_NETWORKS[i].ssid);
  }

  Serial.println("[WiFi] Connecting to Station network...");
  unsigned long t0 = millis();
  while (wifiMulti.run() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(400);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Station Connected Successfully!");
    Serial.printf("  Connected SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("  Station IP URL: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Gateway:        %s\n", WiFi.gatewayIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] Station connection timed out. SoftAP is active at http://192.168.4.1");
  }

  // 4. Start mDNS Responder (http://biped.local)
  if (MDNS.begin(MDNS_HOSTNAME)) {
    Serial.printf("[mDNS] Responder started: http://%s.local\n", MDNS_HOSTNAME);
    MDNS.addService("http", "tcp", 80);
  }

  // 5. Register HTTP Routes
  server.on("/", handleRoot);
  server.on("/get_angles", handleGetAngles);
  server.on("/run_gait", handleRunGait);
  server.on("/set_servo", handleSetServo);
  server.on("/set_gait_param", handleSetGaitParam);
  server.on("/save_offsets", handleSaveOffsets);
  server.on("/reset_offsets", handleResetOffsets);
  server.on("/scan_i2c", handleScanI2C);

  server.begin();
  Serial.println("[HTTP] Web Server Started on Port 80!\n");
}

void handleWebClients() {
  server.handleClient();
}

void printNetworkStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[NET STATUS] Station: http://%s (SSID: %s) | SoftAP: http://%s\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.SSID().c_str(),
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.printf("[NET STATUS] SoftAP Active: http://%s (Connect WiFi to '%s' pass '%s')\n",
                  WiFi.softAPIP().toString().c_str(),
                  AP_SSID, AP_PASS);
  }
}
