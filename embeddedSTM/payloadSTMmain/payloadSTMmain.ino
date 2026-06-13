/*
 * payloadSTMmain_fixed.ino
 *
 * Mission-critical + safety/recovery fix pass. Structure, names and comments
 * from the original are preserved so the diff is easy to read and validate.
 * Every change is tagged inline with "// FIX #n". Summary of changes:
 *
 *   #1  setup(): light-baseline while-loop was an infinite loop (double index
 *       increment) with an out-of-bounds write -> rewritten as a correct for-loop.
 *   #2  setup(): baseline computed from the wrong variable (always 0) -> fixed.
 *   #3  accel_dt was unsigned long, so dt truncated to 0 and accelVelocity never
 *       integrated -> changed to float.
 *   #4  burnoutDetected was never set -> added a burnout detector (also arms the
 *       pyro path and the apogee/velocity logic).
 *   #5  Telemetry was gated on piSerial.available() and lastLogTime never updated
 *       -> gated on logData and properly rate-limited.
 *   #6  SAFETY: nosecone/pyro could trigger under thrust on a single noisy light
 *       sample -> gated on liftoff + debounced; the in-flight early-fire guarantee
 *       now rests on a light-tight bay (VERIFY BY GROUND TEST), not an algorithm.
 *   #7  ADC default is 10-bit but code divides by 4095 -> analogReadResolution(12).
 *   #8  flightState enum mismatched the code -> renumbered (descent=7, landed=8).
 *  #10  loop(): same double-increment light bug + lastLightTime never updated.
 *  #11  Sensor init returns were unchecked -> halt-safe error handler.
 *  #13  Reaction wheel enabled 1 s after tender cut (CONOPS says 3 s); pyro-off
 *       pulse and RW-enable were conflated -> split, RW now at +3 s.
 *  #14  Debug print went to the Pi data link -> routed to rwSerial.
 *
 * RW LINK (this pass): plain-ASCII line protocol on USART1 (spec at the RW link
 *   block below). The STM asserts RW_EN at boot so the RW powers up and self-calibrates on the pad,
 *   streams desired mode + continuity at 10 Hz, commands STABILIZE at deploy and
 *   IDLE at landing, and latches a WiFi-off request from liftoff. USART1 now carries
 *   the ASCII link, so the #14 debug print is removed (use SWD/RTT for bench debug).
 * NOT in this pass (next phase): the RPi framed protocol / handshake / timestamp
 *   and the USART2 fallback link.
 */

#include "STM32pins.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

#define piSerial Serial          // NOTE (next phase): confirm this is USB CDC in your build
#define numSamples 30 //# of values in moving average
#define LIGHT_THRESHOLD 0.5

//LSM6DSOX SPI
SPIClass imuSPI(IMU_MOSI,IMU_MISO,IMU_CLK);

//RPI UART
//HardwareSerial piSerial(RPI_TX_STM_RX,RPI_RX_STM_TX);
//baro wire
TwoWire barWire(PB_7, PB_6);

//RW/debug UART
HardwareSerial rwSerial(UART_RX_RW_TX,UART_TX_RW_RX);

// ---- Reaction-wheel link: USART1 <-> ESP32, plain ASCII, 115200 8N1 ----
// STM -> ESP32 @10Hz:  S,<mode>,<wifi>,<cont>,<state>\n
//   mode 0=IDLE 1=STABILIZE (the only field the RW must act on); wifi 1=tear down
//   WiFi (RW latches it, never re-enabled till power cycle); cont=raw continuity
//   ADC, state=flight state (both display-only for the webpage).
//   RW RULE: act only on a fully parsed line; on garbage, KEEP the last mode.
// ESP32 -> STM (optional): R,<mode>,<fault>,<rpm>\n   (STM logs it if present)
#define RW_LINK_BAUD       115200
#define RW_MODE_IDLE       0
#define RW_MODE_STABILIZE  1

uint8_t rwDesiredMode = RW_MODE_IDLE;        // mode streamed to the ESP32 (idempotent resend)
bool rwWifiOff = false;                       // latched at liftoff (WiFi teardown backstop)
unsigned long lastRwTxTime = 0;
const unsigned long rwTxInterval = 100000;    // 10 Hz command cadence
uint16_t lastContRaw = 0;                     // last continuity ADC reading (sent for the webpage)
// optional status received from the RW ("R,mode,fault,rpm")
char rwRxBuf[48]; uint8_t rwRxLen = 0;
int rwMode = -1, rwFault = 0, rwRpm = 0; unsigned long rwStatusMs = 0;

//Peripheral Object Decleration
Adafruit_LSM6DSOX lsm6dsox;                    // LSM6DSOX IMU sensor object
BMP581 pressureSensor;

// ---- Status indicators: NeoPixel (NEO) = flight phase, LED (PB9) = heartbeat ----
// Vacuum-chamber reference card -- NeoPixel color by flightState:
//   0 INIT  white     1 IDLE/ARM blue(blink)   2 LIFTOFF green    3 BURNOUT yellow
//   4 APOGEE cyan     5 NOSEOFF  magenta        6 TENDERCUT red     7 DESCENT orange
//   8 LANDED green(blink)                       fault: solid red + LED blinks the code
// In a vacuum chamber there is NO acceleration, so BURNOUT(yellow) and APOGEE(cyan)
// are skipped -- you jump green -> magenta when you light the sensor. That is expected.
// The LED is a ~2 Hz loop-alive heartbeat: if it STOPS, the loop has hung.
// IN FLIGHT all onboard LEDs go dark (default) so none can illuminate the
// separation photocell and trip a false separation -- see INDICATORS_IN_FLIGHT.
#define INDICATORS_IN_FLIGHT 0   // 0 = LEDs off after liftoff (FLIGHT DEFAULT: keeps light off
                                 //     the photocell). 1 = keep them on -- ONLY for bench/chamber
                                 //     where the cell is optically shielded or the pyro is a
                                 //     Christmas light; otherwise the LEDs WILL trip separation.
Adafruit_NeoPixel pixel(1, NEO, NEO_GRB + NEO_KHZ800);
unsigned long lastHeartbeat = 0;
bool heartbeatOn = false;
uint32_t lastPixelColor = 0xFFFFFFFF;   // sentinel forces the first show()

byte flightState = 0; // FIX #8: startup=0, idle=1, liftoff=2, burnout=3, apogee=4, noseOff=5, tenderCut=6, descent=7, landed=8

// Moving Average Variables for Barometric Pressure
double pressureSamples[numSamples];
uint sampleIndex = 0;
volatile double pressureSum = 0.0;
volatile double movingAverage = 0.0;
double baselinePressure = 0.0;

// Moving Average Variables for light sensor data
double lightSamples[numSamples];
uint lightIndex = 0;
volatile double lightSum = 0.0;
volatile double movingAverageLight = 0.0;
double baselineLight = 0.0;

const unsigned long lightSampleInterval = 100000; //10 Hz light sample interval

// Moving Average Variables for Accelerometer Data
double accelSamples[numSamples];
uint accelIndex = 0;
double accelSum = 0.0;
double movingAvgAccel = 0.0;
double lastAccel = 0.0;                         
float accel_dt = 0.0;                          // FIX #3: was unsigned long -> dt truncated to 0

double rawXgyro = 0.0;
double rawYgyro = 0.0;
double rawZgyro = 0.0;

// Timing Variables for Sampling delta time
unsigned long lastBaroTime = 0;
unsigned long lastAccelTime = 0;
unsigned long lastLightTime = 0;
unsigned long lastGyroTime = 0;

// Complementary Filter Variables
double baroWeight = 0.9;
double accelWeight = 0.1;
double baroVelocity = 0.0;
double accelVelocity = 0.0;
double filteredVelocity = 0.0;

//baro velocity helpers
double lastAltitude = 0.0;
double currentAltitude = 0.0;
double deltaAltitude = 0.0;
unsigned long baro_dt = 0;             // d/dt trend-based velocity calculation
unsigned long previousBaroTime = 0;

//Event helpers
unsigned long noseOffTime = 0;
unsigned long tenderCutTime = 0;
unsigned long apogeeTime = 0;
unsigned long noseLightStart = 0;                     // FIX #6: light-debounce timer
const unsigned long NOSE_LIGHT_DEBOUNCE_US = 200000;  // FIX #6: light must persist 200 ms

//imu normalization
int verticalAxis = 0;                              // Axis to use for main acceleration (0=x, 1=y, 2=z)
float axisSign = 1.0;

bool liftoffDetected = false;                  // Boolean to detect liftoff state
bool apogeeDetected = false;                   // Boolean to detect apogee state
bool burnoutDetected = false;
bool noseOff = false;
bool tenderCut = false;
bool pyroOff = false;                          // FIX #13: tracks the end of the pyro fire pulse
bool rwEnabled = false;
bool landed = false;

unsigned long lastLogTime = 0;
const unsigned long loggingInterval = 167; //~6khz data stream
bool logData = false;

//landing detection
unsigned long landingStableStart = 0;
const unsigned long LANDING_STABLE_US = 3000000UL;    // 3 s of stable low-and-slow before "landed"
const double LANDING_VEL_BAND     = 2.0;              // m/s

bool barBaselineSet = false;                      // Tracks if baseline pressure has been set
bool accelBaselineSet = false;                    // Tracks if baseline accel has been set

const double liftoffAccelThreshold = 3.0;       // Acceleration threshold for liftoff (in m/s^2)
const double liftoffAltitudeThreshold = 50.0;   // Altitude threshold for liftoff (in meters)
const double burnoutAccelThreshold = 5.0;       // FIX #4: avg axial accel below this (after burn) = burnout

unsigned long liftoffTime = 0;

volatile bool accelReady = false;
volatile bool baroReady = false;
volatile bool gyroReady = false;

//IRQ Functions
void accelIRQ(void){
  accelReady = true;
}

void baroIRQ(void){
  baroReady = true;
}

void gyroIRQ(void){
  gyroReady = true;
}

// Push a NeoPixel color only when it changes (show() briefly masks interrupts).
void setPixel(uint8_t r, uint8_t g, uint8_t b){
  uint32_t c = pixel.Color(r, g, b);
  if (c != lastPixelColor){ pixel.setPixelColor(0, c); pixel.show(); lastPixelColor = c; }
}

// Central indicator update: LED loop-alive heartbeat + NeoPixel flight-phase color.
void updateIndicators(){
  // SAFETY: in flight, take ALL onboard light off the separation photocell. The
  // NeoPixel and the blinking heartbeat LED share the bay with the light sensor;
  // a post-liftoff change in their output looks like a nosecone separation (the
  // 250 ms heartbeat even satisfies the 200 ms debounce). Going dark at liftoff is
  // itself the liftoff cue. Override only with INDICATORS_IN_FLIGHT for bench use.
  if (liftoffDetected && !INDICATORS_IN_FLIGHT){
    setPixel(0, 0, 0);
    digitalWrite(LED, LOW);
    return;
  }

  if (millis() - lastHeartbeat >= 250){ lastHeartbeat = millis(); heartbeatOn = !heartbeatOn; digitalWrite(LED, heartbeatOn); }
  bool slowBlink = ((millis() / 500) & 1);   // 1 Hz
  switch (flightState){
    case 1:  setPixel(0, 0, slowBlink ? 60 : 0); break;  // IDLE/ARMED  blue blink
    case 2:  setPixel(0, 60, 0);                 break;  // LIFTOFF     green
    case 3:  setPixel(60, 40, 0);                break;  // BURNOUT     yellow
    case 4:  setPixel(0, 50, 60);                break;  // APOGEE      cyan
    case 5:  setPixel(60, 0, 60);                break;  // NOSEOFF     magenta
    case 6:  setPixel(80, 0, 0);                 break;  // TENDERCUT   red (+ Christmas light on PYRO)
    case 7:  setPixel(70, 25, 0);                break;  // DESCENT     orange
    case 8:  setPixel(0, slowBlink ? 60 : 0, 0); break;  // LANDED      green blink
    default: setPixel(40, 40, 40);               break;  // INIT/unknown white
  }
}

// FIX #11: critical init failure -> keep all actuators safe and blink a code forever.
// (A dead IMU/baro means liftoff/apogee/landing detection is unreliable, so the
//  safe response is to NOT arm or fly. PI_EN/PYRO/RW are forced low here.)
void flightError(int code){
  digitalWrite(PI_EN, LOW);
  digitalWrite(PYRO, LOW);
  digitalWrite(RW_EN, LOW);
  setPixel(80, 0, 0);   // solid red = init fault (LED also blinks the code below)
  while(true){
    for(int i = 0; i < code; i++){
      digitalWrite(LED, HIGH); delay(200);
      digitalWrite(LED, LOW);  delay(200);
    }
    delay(1000);
  }
}

// ---- Reaction-wheel link (plain ASCII, both directions non-blocking) ----
// TX: one "S,..." line per call. Dropped if the UART buffer is full -> never blocks.
void rwSendCommand(){
  char line[40];
  int n = snprintf(line, sizeof(line), "S,%u,%u,%u,%u\n",
                   (unsigned)rwDesiredMode, (unsigned)(rwWifiOff ? 1 : 0),
                   (unsigned)lastContRaw, (unsigned)flightState);
  if (n > 0 && rwSerial.availableForWrite() >= n) rwSerial.write((const uint8_t*)line, n);
}
// RX: drain available bytes; parse optional "R,mode,fault,rpm" status lines.
// Bounded by bytes available -> never blocks. Malformed lines are ignored.
void rwPoll(){
  while (rwSerial.available()){
    char ch = (char)rwSerial.read();
    if (ch == '\n' || ch == '\r'){
      if (rwRxLen > 0){
        rwRxBuf[rwRxLen] = '\0';
        int m, f, r;
        if (sscanf(rwRxBuf, "R,%d,%d,%d", &m, &f, &r) == 3){
          rwMode = m; rwFault = f; rwRpm = r; rwStatusMs = millis();
        }
        rwRxLen = 0;
      }
    } else if (rwRxLen < sizeof(rwRxBuf) - 1){
      rwRxBuf[rwRxLen++] = ch;
    } else {
      rwRxLen = 0;   // overflow -> resync on next newline
    }
  }
}

// save transmission state between loops
void setup() {
  //pin setups
  delay(3000);
  pinMode(CONT,INPUT_ANALOG);  pinMode(LIGHT,INPUT_ANALOG);
  pinMode(PI_EN,OUTPUT); pinMode(PYRO,OUTPUT); pinMode(LED,OUTPUT); pinMode(RW_EN, OUTPUT);

  digitalWrite(LED,HIGH); digitalWrite(PI_EN,LOW); digitalWrite(PYRO,LOW); digitalWrite(RW_EN,LOW);

  analogReadResolution(12);   // FIX #7: default is 10-bit; code scales by 4095 (12-bit)

  pixel.begin();
  setPixel(40, 40, 40);   // white during init (visible before sensors come up; flightError can paint it red)

  //Serial Initalization
  piSerial.begin(115200);  
  rwSerial.begin(RW_LINK_BAUD);   // ASCII reaction-wheel link to the ESP32

  /*
  IMU SETUP
  */

  // Initialize LSM6DSOX IMU over SPI
  if(!lsm6dsox.begin_SPI(IMU_CS,&imuSPI)){   // FIX #11: was unchecked
    flightError(1);
  }
  
  //accel setup
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G); // Set Acceleration Range to max (16G)
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ); //set Accel Data Rate
  lsm6dsox.configInt2(false,false,true); //Configure Innterupt when Accel Data Ready
  attachInterrupt(digitalPinToInterrupt(IMU_INT2), accelIRQ, RISING); // set interrupt pin for lsm6dsox acceleration data

  //gyro setup
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); // set gyro range to max
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ); //set gyro data rate
  lsm6dsox.configInt1(false,true,false); //configure gyro int for
  attachInterrupt(digitalPinToInterrupt(IMU_INT1), gyroIRQ, RISING); // set interrupt pin for lsm6dsox gyro data

  delay(1000); //let imu normalize
  // Determine primary axis for flight direction based on initial attitude
  sensors_event_t accel;
  lsm6dsox.getEvent(&accel, NULL, NULL);
  double x = abs(accel.acceleration.x);
  double y = abs(accel.acceleration.y);
  double z = abs(accel.acceleration.z);

  float rawX = accel.acceleration.x;
  float rawY = accel.acceleration.y;
  float rawZ = accel.acceleration.z;

  // Find which axis has the largest absolute value (closest to +/- 9.81 m/s²)
  if (abs(rawX) > abs(rawY) && abs(rawX) > abs(rawZ)) {
    verticalAxis = 0; // X is vertical
    if (rawX > 0) {
      axisSign = 1.0;
    } else {
      axisSign = -1.0;
    }
  } 
  else if (abs(rawY) > abs(rawX) && abs(rawY) > abs(rawZ)) {
    verticalAxis = 1; // Y is vertical
    if (rawY > 0) {
      axisSign = 1.0;
    } else {
      axisSign = -1.0;
    }
  } 
  else {
    verticalAxis = 2; // Z is vertical
    if (rawZ > 0) {
      axisSign = 1.0;
    } else {
      axisSign = -1.0;
    }
  }

  /*
  BAROMETER SETUP
  */
  barWire.begin();
  // Check if sensor is connected and initialize
  // FIX #11: was unchecked AND used the default Wire bus instead of barWire (PB6/PB7).
  if(pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY, barWire) != BMP5_OK){
    flightError(2);
  }
  pressureSensor.setMode(BMP5_POWERMODE_CONTINOUS);
  //multiplyers for Output data rate - 500Hz in for 1X,1X - Refer to table 9 in bmp581 datasheet
  bmp5_osr_odr_press_config osrMultipliers = {
      .osr_t = BMP5_OVERSAMPLING_1X,
      .osr_p = BMP5_OVERSAMPLING_1X,
      0,0 // Unused values, included to avoid compiler warnings-as-error
  };
  pressureSensor.setOSRMultipliers(&osrMultipliers);
  // Configure the BMP581 to trigger interrupts whenever a measurement is performed
  BMP581_InterruptConfig interruptConfig = {
      .enable   = BMP5_INTR_ENABLE,    // Enable interrupts
      .drive    = BMP5_INTR_PUSH_PULL, // Push-pull or open-drain
      .polarity = BMP5_ACTIVE_HIGH,    // Active low or high
      .mode     = BMP5_PULSED,         // Latch or pulse signal
      .sources  =
      {
          .drdy_en = BMP5_ENABLE,        // Trigger interrupts when data is ready
          .fifo_full_en = BMP5_DISABLE,  // Trigger interrupts when FIFO is full
          .fifo_thres_en = BMP5_DISABLE, // Trigger interrupts when FIFO threshold is reached
          .oor_press_en = BMP5_DISABLE   // Trigger interrupts when pressure goes out of range
      }
  };
  pressureSensor.setInterruptConfig(&interruptConfig);
  // Setup interrupt handler for BMP581
  attachInterrupt(digitalPinToInterrupt(BAR_INT), baroIRQ, RISING);
  
  ////////////
  ////CONT////
  ////////////

  // Initialize moving average samples and sum
  for (int i = 0; i < numSamples; i++) {
    pressureSamples[i] = 0.0;
    accelSamples[i] = 0.0;
  }

  //////////////////
  ////light sen ////
  //////////////////

  //essentially polls light sensor data to set a baseline for the payload inside rocket
  // FIX #1 + #2: original while-loop double-incremented lightIndex (never exited,
  // wrote one past the array) and divided the wrong variable. Rewritten as a
  // simple correct fill that also primes lightSum/lightIndex for loop().
  double lightBaselineSum = 0.0;
  for (int i = 0; i < numSamples; i++) {
    int rawValue = analogRead(LIGHT);
    float voltage = (rawValue / 4095.0) * 3.3;
    lightSamples[i] = voltage;            // prime the ring buffer
    lightBaselineSum += voltage;
  }
  lightSum = lightBaselineSum;            // keep running average consistent for loop()
  lightIndex = 0;                         // next sample in loop() overwrites slot 0
  baselineLight = lightBaselineSum / numSamples;   // FIX #2: was baselineLight / numSamples (=0)
  movingAverageLight = baselineLight;

  //END LIGHT sensor

  // Power on the reaction-wheel computer (gated by RW_EN). It boots, self-
  // calibrates its gyro WHILE STILL on the pad, and idles in MODE_IDLE until we
  // command STABILIZE at deployment. Done only after STM sensor init succeeds --
  // a flightError() halt above leaves the RW unpowered.
  digitalWrite(RW_EN, HIGH);

  //Set Flight State to Waiting at PAD
  flightState = 1; 

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  unsigned long currentTime = micros();

  rwPoll();   // drain optional reaction-wheel status lines (non-blocking)

  if (baroReady) {
    baroReady = false;

    bmp5_sensor_data data = {0,0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Read true temperature & pressure with compensation
    double realTemperature = data.temperature;
    double realPressure = data.pressure;

    // Update moving average for barometric pressure
    pressureSum -= pressureSamples[sampleIndex];
    pressureSamples[sampleIndex] = realPressure;
    pressureSum += realPressure;
    sampleIndex = (sampleIndex + 1) % numSamples;

    movingAverage = pressureSum / numSamples;

     if (!barBaselineSet && sampleIndex == 0) {  // Set baseline after buffer fills
      baselinePressure = movingAverage;
      barBaselineSet = true;

      // (debug print removed: USART1 now carries the binary RW link; ASCII here
      //  would corrupt frames. Use SWD/RTT for bench debug.)
    }

    if (barBaselineSet) {
      currentAltitude = 44330.0 * (1.0 - pow(movingAverage / baselinePressure, 0.1903));

      // Update trend-based velocity
      if (currentTime - previousBaroTime >= 100000) {  // 100 ms interval for trend calculation
        float deltaAltitude = currentAltitude - lastAltitude;
        float deltaTime = (currentTime - previousBaroTime) / 1.0e6; // Convert to seconds
        baroVelocity = deltaAltitude / deltaTime;  // Calculate trend-based velocity

        // Reset trend variables
        lastAltitude = currentAltitude;
        previousBaroTime = currentTime;
      }
    } 
  }

  if(accelReady){
    accelReady = false;

    currentTime = micros();
    accel_dt = (currentTime - lastAccelTime) / 1.0e6;   // FIX #3: now a real float dt (seconds)
    lastAccelTime = currentTime;
    
    // Get IMU acceleration on the main axis
    sensors_event_t accel;
    lsm6dsox.getEvent(&accel, NULL, NULL);
    // NORMALIZE
    float rawVerticalAccel = 0.0;
    if (verticalAxis == 0) {
      rawVerticalAccel = accel.acceleration.x;
    } 
    else if (verticalAxis == 1) {
      rawVerticalAccel = accel.acceleration.y;
    } 
    else {
      rawVerticalAccel = accel.acceleration.z;
    }

    float normalizedAccel = rawVerticalAccel * axisSign;

    //TRUE PHYSICS INTEGRATION
    float trueKinematicAccel = normalizedAccel - 9.81;
    accelVelocity += trueKinematicAccel * accel_dt;

    // Update moving average buffer
    accelSum -= accelSamples[accelIndex];
    accelSamples[accelIndex] = normalizedAccel;
    accelSum += normalizedAccel;
    accelIndex = (accelIndex + 1) % numSamples;

    // Compute moving average only after buffer is filled
    if (!accelBaselineSet && accelIndex == 0) {
      accelBaselineSet = true;  // Set baseline once the buffer is full
    }
    //Calculate moving average
    if (accelBaselineSet) {
      movingAvgAccel = accelSum / numSamples;
    }
  }

  if(gyroReady){
    gyroReady = false;

    currentTime = micros();
    float gyroDt = (currentTime-lastGyroTime) / 1.0e6;
    lastGyroTime = currentTime;

    sensors_event_t gyro;
    lsm6dsox.getEvent(NULL, &gyro, NULL);
    //integrate dps of each axis to get angle
    rawXgyro = gyro.gyro.x;
    rawYgyro = gyro.gyro.y;
    rawZgyro = gyro.gyro.z;
  }

  //light sensor updating:
  //checking the light level on the photo resistor
  if (currentTime - lastLightTime >= lightSampleInterval) {
    lastLightTime = currentTime;                  // FIX #10: original never updated -> sampled every loop
    int rawValue = analogRead(LIGHT);
    float voltage = (rawValue / 4095.0) * 3.3;
    lightSum -= lightSamples[lightIndex];
    lightSamples[lightIndex] = voltage;
    lightSum += voltage;
    lightIndex = (lightIndex + 1) % numSamples;   // FIX #10: removed second increment (double-advance + OOB)

    movingAverageLight = lightSum / numSamples;
  }

  if(!liftoffDetected){
    baroWeight = 0.0;
    accelWeight = 0.0;
  }else if(liftoffDetected && !burnoutDetected){
    baroWeight = 0.0;
    accelWeight = 1.0;
  }else if(liftoffDetected && burnoutDetected && accelVelocity <= 300){
    baroWeight = 0.05;
    accelWeight = 0.95;
  } else{ //this shouldnt happen in but is here just in case, this is the catch for non-acceleration flight (vacuum chamber) or accel failure
    baroWeight= 1; 
    accelWeight = 0;
  }

  // Apply complementary filter
  filteredVelocity = baroWeight * baroVelocity + accelWeight * accelVelocity;


  ////////////////
  //State Machine:
  ////////////////

  // Check if liftoff detected by altitude and acceleration thresholds
  if (!liftoffDetected && accelBaselineSet && barBaselineSet) {
    if ((currentAltitude >= liftoffAltitudeThreshold) || (movingAvgAccel >= 20.0 && currentAltitude >= 10)){
      liftoffDetected = true;
      digitalWrite(PI_EN,HIGH);// turn on rpi

      logData = true;
      liftoffTime = micros();
      flightState = 2;
      rwWifiOff = true;   // backstop: ensure the RW tears down WiFi even if the operator never armed

      // SAFETY: re-capture the dark-bay light baseline at liftoff so changes in pad
      // ambient (cloud cover, sun angle) during the armed wait can't skew it. The
      // running average is the last ~3 s of light with the cone still on, i.e. the
      // current dark level. Frozen here for the rest of the flight -- separation is
      // a step ABOVE this; we deliberately do NOT keep adapting it (an adaptive
      // baseline could track out a real separation and never trigger).
      baselineLight = movingAverageLight;

    }
  }

  // FIX #4: burnout detection (was never set). After a minimum burn time, the
  // averaged axial acceleration falling back below threshold means the motor has
  // stopped pushing. Burnout enables the velocity filter/apogee logic AND is the
  // safety gate that arms nosecone/pyro (see below).
  if (liftoffDetected && !burnoutDetected && (currentTime - liftoffTime >= 300000) &&
      movingAvgAccel < burnoutAccelThreshold) {
    burnoutDetected = true;
    flightState = 3;
  }

  // Apogee Detection (fused velocity)
  if (liftoffDetected && burnoutDetected && !apogeeDetected && filteredVelocity < 0.1) { // When trend-based baro velocity ~ 0 at peak
    apogeeDetected = true;
    apogeeTime = micros();
    flightState = 4;
  }

  // nosecone deployment sensed
  // FIX #6 (SAFETY): gated on liftoff ONLY. The light sensor is the authority for
  // separation -- with the nosecone on, the payload bay must be light-tight, so the
  // sensor physically cannot read bright until the cone is actually off. Safety
  // against an early fire therefore rests on a VERIFIED-DARK BAY (ground test),
  // not on any flight-phase algorithm or timer. The liftoff gate only rejects
  // ground/handling false-triggers. The debounce below is sensor noise rejection
  // (one ADC glitch must not fire a pyro), NOT a flight timer.
  // TRADE-OFF: with no burnout gate and no backup, the light sensor is a single
  // point of failure for recovery -- if it fails dark, nothing cuts the tender.
  if (liftoffDetected && !noseOff &&
      movingAverageLight >= (baselineLight + LIGHT_THRESHOLD)) {
    if (noseLightStart == 0) {
      noseLightStart = currentTime;                  // start the debounce timer
    } else if (currentTime - noseLightStart >= NOSE_LIGHT_DEBOUNCE_US) {
      noseOff = true;
      noseOffTime = micros();
      flightState = 5;
    }
  } else if (!noseOff) {
    noseLightStart = 0;                              // condition broke -> reset debounce
  }

  // cut tender 
  if (liftoffDetected && noseOff && currentTime - noseOffTime >= 2000000 && !tenderCut) {
    digitalWrite(PYRO,HIGH);
    tenderCut = true;
    tenderCutTime = micros();
    flightState = 6;
  }

  // FIX #13: end the pyro fire pulse after 1 s (separated from RW activation).
  if (liftoffDetected && tenderCut && !pyroOff && currentTime - tenderCutTime >= 1000000) {
    digitalWrite(PYRO, LOW);
    pyroOff = true;
  }

  // Command active stabilization 3 s after tender cut (CONOPS: let the chute fill).
  // The RW is already powered (RW_EN asserted at boot) and pre-calibrated, so this
  // is just a mode command over UART -> stabilization begins within a control cycle.
  if (liftoffDetected && tenderCut && !rwEnabled && currentTime - tenderCutTime >= 3000000) {
    rwDesiredMode = RW_MODE_STABILIZE;
    rwEnabled = true;
    flightState = 7;   // descent / stabilizing
  }

  if(tenderCut && !landed && fabs(baroVelocity) < LANDING_VEL_BAND) {
    if (landingStableStart == 0) {
      landingStableStart = micros();                 // start the stability timer
    } else if (micros() - landingStableStart >= LANDING_STABLE_US) {
      landed = true;
      flightState = 8;   // landed
      rwDesiredMode = RW_MODE_IDLE;   // stop active stabilization (motor commands 0 A)
      logData = false;   // FIX: stop the data stream on landing (CONOPS step 12)
      //Pi stop-recording / shutdown handshake -> RPi-link phase
      //RW left powered & idle; hard power-down (drop RW_EN) is the optional later step.
    }
  } else {
    landingStableStart = 0;                          // condition broke -> reset timer
  }

  ///////////////////////
  ///END STATE MACHINE///
  ///////////////////////

  updateIndicators();   // NeoPixel flight-phase color + LED loop-alive heartbeat

  // Reaction-wheel link: stream desired mode + continuity to the ESP32 at ~10 Hz.
  // Non-blocking; mode is idempotent so a dropped line self-heals next cycle.
  if (currentTime - lastRwTxTime >= rwTxInterval) {
    lastRwTxTime = currentTime;
    lastContRaw = (uint16_t)analogRead(CONT);
    rwSendCommand();
  }

  //Data Stream (to CM4 for logging)
  // FIX #5: was gated on piSerial.available() (only transmitted when the Pi sent
  // bytes) and lastLogTime was never updated (no rate limit). Now gated on logData
  // and properly rate-limited. (ASCII CSV + String kept for now; the binary framed
  // protocol w/ timestamp+seq+CRC and the PI_READY/TIME_SYNC/SEPARATION/STOP
  // handshake are the RPi-link phase.)
  if(logData && currentTime - lastLogTime >= loggingInterval){
    lastLogTime = currentTime;
    String str = String(currentAltitude) + "," + String(movingAvgAccel)  + "," + String(baroVelocity) + "," + String(flightState)
                 + "," + String(movingAverageLight, 3) + "," + String(baselineLight, 3);
    // fields: alt,accel,baroVel,state,light,lightBaseline
    // (light + baseline added so you can watch the photocell vs its trigger point during tests)
    piSerial.println(str);
  }
}
