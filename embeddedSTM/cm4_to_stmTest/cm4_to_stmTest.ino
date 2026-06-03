// CM4 power-on -> serial-hello timing test (STM32F411, stm32duino)
// Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)"

#include <string.h>

#define UART_TX_RW_RX PA9
#define UART_RX_RW_TX PA10
HardwareSerial debugSerial(UART_RX_RW_TX, UART_TX_RW_RX);

#define PI_PWR_EN     PC13     // <-- your regulator-enable GPIO
#define PI_ON_LEVEL   HIGH    // <-- level that ENABLES the 5V reg (HIGH or LOW)
#define PI_OFF_LEVEL  (PI_ON_LEVEL == HIGH ? LOW : HIGH)

#define OFF_SETTLE_MS 3000UL    // hold off this long so the Pi fully discharges
#define HELLO_TIMEOUT 120000UL  // give up waiting after this long

void piPower(bool on) {
  digitalWrite(PI_PWR_EN, on ? PI_ON_LEVEL : PI_OFF_LEVEL);
}

// Watch USB CDC for a line containing "hello". True if seen before timeout.
bool waitForHello(uint32_t timeout_ms) {
  char buf[16];
  uint8_t len = 0;
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        buf[len] = 0;
        if (strstr(buf, "hello")) return true;
        len = 0;
      } else if (len < sizeof(buf) - 1) {
        buf[len++] = c;
      } else {
        len = 0;  // overflow, resync
      }
    }
  }
  return false;
}

void runMeasurement() {
  piPower(false);
  debugSerial.print("Pi OFF, settling "); debugSerial.print(OFF_SETTLE_MS);
  debugSerial.println(" ms...");
  delay(OFF_SETTLE_MS);

  while (Serial.available()) Serial.read();  // drop stale bytes

  uint32_t t0 = millis();
  piPower(true);
  debugSerial.println("Pi POWER ON - waiting for 'hello'...");

  if (waitForHello(HELLO_TIMEOUT)) {
    uint32_t dt = millis() - t0;
    debugSerial.print(">>> hello after ");
    debugSerial.print(dt); debugSerial.print(" ms  (");
    debugSerial.print(dt / 1000.0, 2); debugSerial.println(" s)");
  } else {
    debugSerial.println(">>> TIMEOUT: no hello received");
  }
  debugSerial.println("Press any key to power-cycle and measure again.");
}

void setup() {
  pinMode(PI_PWR_EN, OUTPUT);
  piPower(false);                  // start with Pi off and known

  Serial.begin(115200);           // USB CDC to the Pi
  debugSerial.begin(115200);      // debug UART

  debugSerial.println();
  debugSerial.println("CM4 boot-to-serial timing test");
  debugSerial.println("Press any key to power on the Pi and start timing.");
}

void loop() {
  if (debugSerial.available()) {
    while (debugSerial.available()) debugSerial.read();  // eat the keypress
    runMeasurement();
  }
}