#include <Arduino.h>
#include "config.h"
#include "pca_driver.h"
#include "kinematics.h"
#include "web_server.h"

// =============================================================================
// FreeRTOS Dedicated Real-Time Locomotion Task (Core 1)
// Runs strictly at 50Hz (every 20ms) with deterministic microsecond precision.
// Fully isolated from WiFi/network interrupts on Core 0.
// =============================================================================
void locomotionTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(LOCOMOTION_PERIOD_MS);

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    updateLocomotion(0.020f);
  }
}

// =============================================================================
// USB Serial Command Listener
// Allows 100% control over USB cable even without any WiFi connection!
// =============================================================================
void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "walk" || cmd == "start" || cmd == "forward") {
    Serial.println("[CLI] Continuous Dynamic Walk Started.");
    startLocomotion("walk");
  } else if (cmd == "step") {
    Serial.println("[CLI] Executing Single Full Human Stride...");
    executeSingleStep();
    Serial.println("[CLI] Stride Complete. Settled in Stand Neutral.");
  } else if (cmd == "takestep" || cmd == "ref_step") {
    Serial.println("[CLI] Executing Reference takeStep(1.6, 20)...");
    takeStep(1.6f, 20);
    stopLocomotion();
    Serial.println("[CLI] Reference takeStep Complete.");
  } else if (cmd == "init" || cmd == "initialize") {
    Serial.println("[CLI] Executing Reference initialize()...");
    initialize();
    Serial.println("[CLI] Reference initialize Complete.");
  } else if (cmd == "stop" || cmd == "stand" || cmd == "park") {
    Serial.println("[CLI] Settling to Calibrated Stand Neutral.");
    stopLocomotion();
  } else if (cmd == "diag" || cmd == "test") {
    Serial.println("[CLI] Running Joint Diagnostics Sweep...");
    runJointDiagnostics();
    Serial.println("[CLI] Diagnostics Sweep Complete.");
  } else if (cmd == "ip" || cmd == "net" || cmd == "wifi") {
    printNetworkStatus();
  } else if (cmd == "status" || cmd == "telemetry") {
    LocomotionTelemetry t = getLocomotionTelemetry();
    Serial.printf("[STATUS] Gait: %s | Running: %s | Contact: %s | Force: %.2f N (L:%.1fg, R:%.1fg)\n",
                  t.activeGait.c_str(), t.isRunning ? "YES" : "NO", t.contactState.c_str(),
                  t.groundForceNewtons, t.weightLeftGrams, t.weightRightGrams);
    Serial.print("  Angles: ");
    for (int i = 0; i < NUM_SERVOS; i++) {
      JointState* j = getJoint(i);
      if (j) Serial.printf("[CH%d %s: %.1f°] ", j->channel, j->name, j->currentAngle);
    }
    Serial.println();
  } else if (cmd == "help" || cmd == "?") {
    Serial.println("\n--- Biped Robot Serial CLI Commands ---");
    Serial.println("  walk   - Start continuous forward biped walking");
    Serial.println("  step   - Execute one single smooth stride");
    Serial.println("  stop   - Emergency stop and hold neutral stand");
    Serial.println("  diag   - Run joint diagnostics safety sweep");
    Serial.println("  ip     - Display active WiFi station & SoftAP URLs");
    Serial.println("  status - Display live 6-joint angles & ground force telemetry");
    Serial.println("  help   - Show this help menu\n");
  }
}

// =============================================================================
// PlatformIO Setup & Main Loop
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n========================================================");
  Serial.println("🤖 ESP32 6-DOF Bipedal Walking Robot Controller");
  Serial.println("   Architecture: PlatformIO (Core 1 Locomotion + Core 0 Network)");
  Serial.println("========================================================\n");

  // 1. Load NVS Calibrated Offsets
  loadNVSJointOffsets();

  // 2. Initialize PCA9685 Servo Driver @ 400kHz Fast I2C
  initPCA();

  // 3. Park Servos to Calibrated Neutral Standing Baseline
  parkNeutralStand();

  // 4. Initialize Kinematics Engine
  initKinematics();

  // 5. Initialize WiFi Station, SoftAP & Embedded Web Server
  initWebServer();

  // 6. Spawn Dedicated Real-Time Locomotion Task on Core 1
  xTaskCreatePinnedToCore(
    locomotionTask,
    "LocomotionTask",
    4096,
    NULL,
    configMAX_PRIORITIES - 1, // Highest priority for judder-free motion
    NULL,
    1                         // Pinned to Core 1
  );

  Serial.println("[SYSTEM] FreeRTOS Locomotion Task started on Core 1 @ 50Hz.");
  Serial.println("[SYSTEM] Type 'help' in Serial Monitor for interactive CLI.\n");
}

void loop() {
  // Core 0 handles web server requests and USB serial commands
  handleWebClients();
  handleSerialCommands();

  // Auto-reconnect PCA9685 watchdog if hardware momentarily resets
  static unsigned long lastPcaCheck = 0;
  if (millis() - lastPcaCheck > 2000) {
    lastPcaCheck = millis();
    checkPCAConnection();
  }

  // Periodic network heartbeat to Serial (every 5 seconds)
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    printNetworkStatus();
  }
}
