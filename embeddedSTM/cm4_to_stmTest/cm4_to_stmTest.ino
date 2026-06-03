// CM4 boot-to-serial timing test + NeoPixel status (STM32F411, stm32duino)
// Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)"
// Library Manager: "Adafruit NeoPixel"

#include <string.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

#define UART_TX_RW_RX PA9
#define UART_RX_RW_TX PA10
HardwareSerial debugSerial(UART_RX_RW_TX, UART_TX_RW_RX);

#define PI_PWR_EN     PB0     // <-- your regulator-enable GPIO
#define PI_ON_LEVEL   HIGH    // <-- level that ENABLES the 5V reg
#define PI_OFF_LEVEL  (PI_ON_LEVEL == HIGH ? LOW : HIGH)

#define NEOPIXEL_PIN  PB4
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define OFF_SETTLE_MS 3000UL
#define HELLO_TIMEOUT 120000UL

enum LedState { LED_IDLE, LED_BOOTING, LED_BOOTED, LED_TIMEOUT };
LedState ledState = LED_IDLE;

void piPower(bool on) {
  digitalWrite(PI_PWR_EN, on ? PI_ON_LEVEL : PI_OFF_LEVEL);
}

// Call this as often as you like; it self-throttles to ~60 fps.
void updateLed() {
  static uint32_t last = 0;
  uint32_t t = millis();
  if (t - last < 16) return;
  last = t;

  switch (ledState) {
    case LED_IDLE: {            // STM alive, Pi off -> slow breathing blue
      float b = sinf(((t % 3000) / 3000.0f) * 2 * PI) * 0.5f + 0.5f;
      pixel.setPixelColor(0, pixel.Color(0, 0, 5 + (uint8_t)(b * 60)));
      break;
    }
    case LED_BOOTING: {         // powered, waiting for hello -> rainbow swirl
      pixel.setPixelColor(0, pixel.gamma32(pixel.ColorHSV((uint16_t)(t * 24), 255, 90)));
      break;
    }
    case LED_BOOTED:            // hello received -> steady green
      pixel.setPixelColor(0, pixel.Color(0, 120, 0));
      break;
    case LED_TIMEOUT: {         // no hello -> pulsing red
      float b = sinf(((t % 700) / 700.0f) * 2 * PI) * 0.5f + 0.5f;
      pixel.setPixelColor(0, pixel.Color(15 + (uint8_t)(b * 120), 0, 0));
      break;
    }
  }
  pixel.show();
}

bool waitForHello(uint32_t timeout_ms) {
  char buf[16];
  uint8_t len = 0;
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    updateLed();
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        buf[len] = 0;
        if (strstr(buf, "hello")) return true;
        len = 0;
      } else if (len < sizeof(buf) - 1) {
        buf[len++] = c;
      } else {
        len = 0;
      }
    }
  }
  return false;
}

void celebrate() {            // quick green sparkle on success
  for (int i = 0; i < 3; i++) {
    pixel.setPixelColor(0, pixel.Color(0, 255, 0)); pixel.show(); delay(70);
    pixel.setPixelColor(0, 0);                      pixel.show(); delay(70);
  }
}

void runMeasurement() {
  ledState = LED_IDLE;        // Pi is off during the settle
  piPower(false);
  debugSerial.print("Pi OFF, settling "); debugSerial.print(OFF_SETTLE_MS);
  debugSerial.println(" ms...");
  uint32_t s = millis();
  while (millis() - s < OFF_SETTLE_MS) updateLed();

  while (Serial.available()) Serial.read();   // drop stale bytes

  uint32_t t0 = millis();
  piPower(true);
  ledState = LED_BOOTING;
  debugSerial.println("Pi POWER ON - waiting for 'hello'...");

  if (waitForHello(HELLO_TIMEOUT)) {
    uint32_t dt = millis() - t0;
    debugSerial.print(">>> hello after ");
    debugSerial.print(dt); debugSerial.print(" ms  (");
    debugSerial.print(dt / 1000.0, 2); debugSerial.println(" s)");
    celebrate();
    ledState = LED_BOOTED;
  } else {
    debugSerial.println(">>> TIMEOUT: no hello received");
    ledState = LED_TIMEOUT;
  }
  debugSerial.println("Press any key to power-cycle and measure again.");
}

void setup() {
  pinMode(PI_PWR_EN, OUTPUT);
  piPower(false);

  pixel.begin();
  pixel.show();               // start dark

  Serial.begin(115200);
  debugSerial.begin(115200);

  debugSerial.println();
  debugSerial.println("CM4 boot-to-serial timing test + NeoPixel");
  debugSerial.println("Press any key to power on the Pi and start timing.");
}

void loop() {
  updateLed();
  if (debugSerial.available()) {
    while (debugSerial.available()) debugSerial.read();
    runMeasurement();
  }
}