#include "STM32pins.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>

#define piSerial Serial
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

//Peripheral Object Decleration
Adafruit_LSM6DSOX lsm6dsox;                    // LSM6DSOX IMU sensor object
BMP581 pressureSensor;

byte flightState = 0; //startup = 0, idle = 1, liftoff = 2, burnout = 3, apoggee = 4, noseOff = 5, tenderCut = 6, landed =7;

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
unsigned long accel_dt = 0;

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

//imu normalization
int verticalAxis = 0;                              // Axis to use for main acceleration (0=x, 1=y, 2=z)
float axisSign = 1.0;

bool liftoffDetected = false;                  // Boolean to detect liftoff state
bool apogeeDetected = false;                   // Boolean to detect apogee state
bool burnoutDetected = false;
bool noseOff = false;
bool tenderCut = false;
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

// save transmission state between loops
void setup() {
  //pin setups
  delay(3000);
  pinMode(CONT,INPUT_ANALOG);  pinMode(LIGHT,INPUT_ANALOG);
  pinMode(PI_EN,OUTPUT); pinMode(PYRO,OUTPUT); pinMode(LED,OUTPUT); pinMode(RW_EN, OUTPUT);

  digitalWrite(LED,HIGH); digitalWrite(PI_EN,LOW); digitalWrite(PYRO,LOW); digitalWrite(RW_EN,LOW);

  //Serial Initalization
  piSerial.begin(115200);  
  rwSerial.begin(115200);

  /*
  IMU SETUP
  */

  // Initialize LSM6DSOX IMU over SPI
  lsm6dsox.begin_SPI(IMU_CS,&imuSPI);
  
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
  pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY);
  // Variable to track errors returned by API calls
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
  bool baslinelightfilled = false;
  //baseline voltage read from the light sensor

  while(!baslinelightfilled)
  {
    int rawValue = analogRead(LIGHT);
    float voltage = 1.0 * (rawValue / 4095.0) * 3.3;

    lightSum -= lightSamples[lightIndex];
    lightSamples[lightIndex] = voltage;
    lightSum += voltage;
    lightIndex = (lightIndex + 1) % numSamples;

    if (lightIndex < numSamples) 
    {
      lightIndex++;
    }else
    {
      baslinelightfilled = true;
    }
  }

  baselineLight = baselineLight / numSamples;
  movingAverageLight = baselineLight;

  //END LIGHT sensor

  //Set Flight State to Waiting at PAD
  flightState = 1; 

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  unsigned long currentTime = micros();

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

      #ifndef FLIGHT
      Serial.println("Baseline pressure set for relative altitude calculation.");
      #endif
    }

    if (barBaselineSet) {
      currentAltitude = 44330.0 * (1.0 - pow(movingAverage / baselinePressure, 0.1903));

      // Update trend-based velocity
      if (currentTime - previousBaroTime >= 100000) {  // 500 ms interval for trend calculation
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
    accel_dt = (currentTime - lastAccelTime) / 1.0e6;
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
    int rawValue = analogRead(LIGHT);
    float voltage = 1.0 * (rawValue / 4095.0) * 3.3;
    lightSum -= lightSamples[lightIndex];
    lightSamples[lightIndex] = voltage;
    lightSum += voltage;
    lightIndex = (lightIndex + 1) % numSamples;

    if (lightIndex < numSamples) 
    {
      lightIndex++;
    }

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
      digitalWrite(LED, LOW);  // Turn on LED for liftoff indication
      digitalWrite(PI_EN,HIGH);// turn on rpi

      logData = true;
      liftoffTime = micros();
      flightState = 2;

    }
  }

  // Apogee Detection (fused velocity)
  if (liftoffDetected && burnoutDetected && !apogeeDetected && filteredVelocity < 0.1) { // When trend-based baro velocity ~ 0 at peak
    apogeeDetected = true;
    apogeeTime = micros();
    digitalWrite(LED,HIGH); //turnoff LED for apoggee detection
    flightState = 4;
  }

  // nosecone deployment sensed
  if (liftoffDetected && movingAverageLight >= (baselineLight + LIGHT_THRESHOLD)){
    noseOff = true;
    noseOffTime = micros();
    flightState = 5;
  }

  // cut tender 
  if (liftoffDetected && noseOff && currentTime - noseOffTime >= 2000000 && !tenderCut) {
    digitalWrite(PYRO,HIGH);
    tenderCut = true;
    tenderCutTime = micros();
    flightState = 6;
  }

  //turnoff tender pyro
  if (liftoffDetected && noseOff && tenderCut && currentTime - tenderCutTime >= 1000000) {
    digitalWrite(PYRO, LOW);
    //Enable active stabilization
    digitalWrite(RW_EN, HIGH);
    rwEnabled = true;
    flightState = 7;
  }

  if(tenderCut && !landed && fabs(baroVelocity) < LANDING_VEL_BAND) {
    if (landingStableStart == 0) {
      landingStableStart = micros();                 // start the stability timer
    } else if (micros() - landingStableStart >= LANDING_STABLE_US) {
      landed = true;
      flightState = 8;
      digitalWrite(LED, LOW);
      digitalWrite(RW_EN, LOW);
      //Pi shutdown
    }
  } else {
    landingStableStart = 0;                          // condition broke -> reset timer
  }

  ///////////////////////
  ///END STATE MACHINE///
  ///////////////////////

  //Data Stream (to CM4 for logging)
  if(piSerial.available() && currentTime - lastLogTime >= loggingInterval){
    String str = String(currentAltitude) + "," + String(movingAvgAccel)  + "," + String(baroVelocity) + "," + String(flightState);
    //add other relevant data such as CONT, light values, gyro would be very cool
    piSerial.println(str);
  }
}

