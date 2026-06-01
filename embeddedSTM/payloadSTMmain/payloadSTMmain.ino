#include "STM32pins.h"

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_LSM6DSOX.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>


//LSM6DSOX SPI
SPIClass SPI_4(LSM_MOSI,LSM_MISO,LSM_CLK);

//RPI UART

//RW/debug UART
//HardwareSerial rcSerial(RX_1_RC_TX,TX_1_RC_RX);


//Peripheral Object Decleration
Adafruit_LSM6DSOX lsm6dsox;                    // LSM6DSOX IMU sensor object
BMP581 pressureSensor;

//Variables:
#define numSamples 30 //# of values in moving average
#define BUFF_SIZE 20


byte flightState = 0; //startup = 0, idle = 1, liftoff = 2, burnout = 3, apoggee & descent under drogue = 4, descent under main = 5, landing detected = 6;

// Moving Average Variables for Barometric Pressure
double pressureSamples[numSamples];
uint sampleIndex = 0;
volatile double pressureSum = 0.0;
volatile double movingAverage = 0.0;
double baselinePressure = 0.0;

// Moving Average Variables for Accelerometer Data
double accelSamples[numSamples];
uint accelIndex = 0;
double accelSum = 0.0;
double movingAvgAccel = 0.0;
double lastAccel = 0.0;                         
unsigned long accel_dt = 0;

// Timing Variables for Sampling delta time
unsigned long lastBaroTime = 0;
unsigned long lastAccelTime = 0;


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


//imu normalization
int mainAxis = 2;                              // Axis to use for main acceleration (0=x, 1=y, 2=z)
double imuFlip = 1.0;
bool imuFlipped = false;

bool liftoffDetected = false;                  // Boolean to detect liftoff state
bool apogeeDetected = false;                   // Boolean to detect apogee state

bool barBaselineSet = false;                      // Tracks if baseline pressure has been set
bool accelBaselineSet = false;                    // Tracks if baseline accel has been set

const double liftoffAccelThreshold = 3.0;       // Acceleration threshold for liftoff (in m/s^2)
const double liftoffAltitudeThreshold = 50.0;   // Altitude threshold for liftoff (in meters)

float basevoltageread= 0.0;                     // lightsen base voltage for detecting nosecone deployment

bool droguePyroActive = false;
bool mainChutePyroActive = false;

bool droguePyroOver = false;
bool mainPyroOver = false;

unsigned long liftoffTime = 0;

volatile bool accelReady = false;

volatile bool baroReady = false;


//IRQ Functions
void accelIRQ(void){
  accelReady = true;
}

void baroIRQ(void){
  baroReady = true;
}

// save transmission state between loops
void setup() {
  //continuity and VBAT Analog pins
  pinMode(CONT1,INPUT_ANALOG); pinMode(CONT2,INPUT_ANALOG); pinMode(VBAT,INPUT_ANALOG);
  //RF Switch Pin

  //Pin Setup

  
  delay(3000); //delay for connecting to Serial Terminal / sensors to start up

  digitalWrite(LED_B,HIGH);


  Serial.begin(115200); //Serial Port (debug)

  //add USB serial to CM4 initialization
  

  Serial.println("payloadSaftey Initializing...");

  /*
  IMU SETUP
  */

  Serial.println("Initializing IMU ...");

  // Initialize LSM6DSOX IMU over SPI
  if (!lsm6dsox.begin_SPI(LSM_CS,&SPI_4)) {
    Serial.println("LSM6DSOX not detected. Check wiring.");
  }

  //accel setup
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G); // Set Acceleration Range to max (16G)
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ); //set Accel Data Rate
  lsm6dsox.configInt2(false,false,true); //Configure Innterupt when Accel Data Ready
  attachInterrupt(digitalPinToInterrupt(LSM_INT2), accelIRQ, RISING); // set interrupt pin for lsm6dsox acceleration data

  Serial.println("IMU - GOOD");

  /*
  BAROMETER SETUP
  */

  Wire.begin();
  // Check if sensor is connected and initialize
  if(pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) != BMP5_OK)
  {
    // Not connected, inform user
    Serial.println("Error: BMP581 not connected, check wiring and I2C address!");
  }

  // Variable to track errors returned by API calls
  int8_t err = BMP5_OK;

  err = pressureSensor.setMode(BMP5_POWERMODE_CONTINOUS);
  if(err != BMP5_OK)
  {
    //errorCode1();
  }

  //multiplyers for Output data rate - 500Hz in for 1X,1X - Refer to table 9 in bmp581 datasheet
  bmp5_osr_odr_press_config osrMultipliers = {
      .osr_t = BMP5_OVERSAMPLING_1X,
      .osr_p = BMP5_OVERSAMPLING_1X,
      0,0 // Unused values, included to avoid compiler warnings-as-error
  };

  err = pressureSensor.setOSRMultipliers(&osrMultipliers);
  if(err)
  {
      //errorCode1();
  }

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

  err = pressureSensor.setInterruptConfig(&interruptConfig);
  if(err != BMP5_OK)
  {
    //errorCode1();
  }

  // Setup interrupt handler for BMP581
  attachInterrupt(digitalPinToInterrupt(BMP_INT), baroIRQ, RISING);

  Serial.println("BARO - GOOD");
  

  ////////////
  ////CONT////
  ////////////
  Serial.println(F("Checking CONT... "));
    if(!(analogRead(CONT1) >= 380)){
        Serial.println(analogRead(CONT1));
        //errorCode1();
  }

  
  Serial.println("Initializing buffers...");


  // Initialize moving average samples and sum
  for (int i = 0; i < numSamples; i++) {
    pressureSamples[i] = 0.0;
    accelSamples[i] = 0.0;
  }

  // Determine primary axis for flight direction based on initial attitude
  sensors_event_t accel;
  lsm6dsox.getEvent(&accel, NULL, NULL);
  double x = abs(accel.acceleration.x);
  double y = abs(accel.acceleration.y);
  double z = abs(accel.acceleration.z);

  if (x > y && x > z) mainAxis = 0;
  else if (y > x && y > z) mainAxis = 1;
  else mainAxis = 2;

  Serial.print("Primary axis for acceleration: ");
  Serial.println(mainAxis == 0 ? "X" : mainAxis == 1 ? "Y" : "Z");

  lsm6dsox.getEvent(&accel, NULL, NULL);

  double acceleration = mainAxis == 0 ? accel.acceleration.x : mainAxis == 1 ? accel.acceleration.y : accel.acceleration.z;

  //////////////////
  ////light sen ////
  //////////////////

  //essentially polls light sensor data to set a baseline for the payload inside rocket
  bool baslinelightfilled = false;
  //baseline voltage read from the light sensor 
  while(!baslinelightfilled)
  {
    int rawValue = analogRead(LIGHT_SENSER);
    float voltage = (rawValue / ADC_RESOLUTION) * REF_VOLTAGE;
    voltageSum -= voltageBuffer[bufferIndex];
    voltageBuffer[bufferIndex] = voltage;
    voltageSum += voltage;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferCount < BUFFER_SIZE) 
    {
      bufferCount++;
    }else
    {
      baslinebarfilled=true;
    }
  }

  basevoltageread = voltageSum / bufferCount;

  Serial.println(basevoltageread);

  Serial.println("FFCV3 Initialized! Awaiting Liftoff....");

  flightState = 1; //Set Flight State to Waiting at PAD

  

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
    // Get IMU acceleration on the main axis
    sensors_event_t accel;
    lsm6dsox.getEvent(&accel, NULL, NULL);

    double acceleration = mainAxis == 0 ? accel.acceleration.x : mainAxis == 1 ? accel.acceleration.y : accel.acceleration.z;

    acceleration *= imuFlip;

    // Update moving average buffer
    accelSum -= accelSamples[accelIndex];
    accelSamples[accelIndex] = acceleration;
    accelSum += acceleration;
    accelIndex = (accelIndex + 1) % numSamples;

    // Integrate raw acceleration for velocity
    accel_dt = micros() - lastAccelTime;
    accelVelocity += (acceleration - 9.81) * accel_dt * 0.000001;
    lastAccelTime = micros();

    // Compute moving average only after buffer is filled
    if (!accelBaselineSet && accelIndex == 0) {
      accelBaselineSet = true;  // Set baseline once the buffer is full
      #ifndef FLIGHT
      Serial.println("IMU baseline set.");
      #endif
    }

    //Calculate moving average and flip if necessary
    if (accelBaselineSet) {
      movingAvgAccel = accelSum / numSamples;

      if(movingAvgAccel > 0 && !imuFlipped){
        imuFlipped = true;
        imuFlip = -1.0;
        #ifndef FLIGHT
        Serial.println("IMU FLIPPED....");
        #endif
      } 
    }
  }

  //light sensor updating:
  /*

  //checking the light level on the photo resistor
  int rawValue = analogRead(LIGHT_SENSER);
  float voltage = (rawValue / ADC_RESOLUTION) * REF_VOLTAGE;

  // Update moving average
  voltageSum -= voltageBuffer[bufferIndex];
  voltageBuffer[bufferIndex] = voltage;
  voltageSum += voltage;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) bufferCount++;

  averageVoltage = voltageSum / bufferCount;
  */

  //^^^^^^^^^^ this need a systick its wasting clock constantly checking maybe 10-100Hz


  //Gain Definitions <- do we even need?
  /*
    if(!liftoffDetected){
    baroWeight = 0.99;
    accelWeight = 0.01;
  }else if(liftoffDetected && filteredVelocity <= 600 && movingAvgAccel >= -6.0){
    baroWeight = 0.3;
    accelWeight = 0.7;
  }else if(liftoffDetected && filteredVelocity > 600 && movingAvgAccel >= -6.0){
    baroWeight = 0.1;
    accelWeight = 0.9;
  } else{ //this shouldnt happen in but is here just in case, this is the catch for non-acceleration flight (vacuum chamber) or accel failure
    baroWeight= .9; 
    accelWeight = .1;
  }
  */


  // Apply complementary filter -- necessary??
  //filteredVelocity = baroWeight * baroVelocity + accelWeight * accelVelocity;


  ////////////////
  //State Machine:
  ////////////////

  // Check if liftoff detected by altitude and acceleration thresholds
  if (!liftoffDetected && accelBaselineSet && barBaselineSet) {
    if ((currentAltitude >= liftoffAltitudeThreshold) || (movingAvgAccel >= (liftoffAccelThreshold - 9.8) && currentAltitude >= 10)){
      liftoffDetected = true;
      digitalWrite(LED_B, LOW);  // Turn on LED for liftoff indication

      #ifndef FLIGHT
      Serial.println("Liftoff detected.");
      #endif
      
      logData = true;
      liftoffTime = micros();
      flightState = 2;

      if(isCamera){
        startRecording();
      }
    }
  }

  // Apogee Detection - technically not needed as we are light sensor based might be worth not even attempting apogee detection and liftoff/nosecone deployment only
  /*
  if (liftoffDetected && !apogeeDetected && filteredVelocity < 0.1) { // When trend-based baro velocity ~ 0 at peak
    apogeeDetected = true;
    apogeeTime = micros();

    #ifndef FLIGHT
    Serial.println("Apogee detected.");
    #endif

    digitalWrite(LED_B,HIGH); //turnoff LED for apoggee detection
    flightState = 4;
  }

  // Drogue pyro activation 1s after apogee detection
  if (apogeeDetected && (micros() - apogeeTime >= apogeeDelay) && !droguePyroActive) {
    digitalWrite(PYRO1_DROG, HIGH);
    droguePyroActive = true;
    drogueActivationTime = micros();

    #ifndef FLIGHT
    Serial.println("Drogue pyro activated for drogue chute.");
    #endif
  }

  // Main chute deployment at target altitude
  if (apogeeDetected && (currentAltitude <= mainChuteAltitude) && !mainChutePyroActive) {
    digitalWrite(PYRO2_MAIN, HIGH);
    mainChutePyroActive = true;
    mainChuteActivationTime = micros();
    #ifndef FLIGHT
    Serial.println("Main chute pyro activated.");
    #endif
    flightState = 5;
  }

  //landing detection
  if (mainChutePyroActive && baroVelocity >= -1) {
    flightState = 6;
    landed = true;

    #ifndef FLIGHT
    Serial.println("landed detected");
    #endif

    digitalWrite(LED_B,LOW);

    if(isCamera){
      stopRecording();
    }

    MyTim->setPWM(channel, BUZ, 4000, 50); // 4kHz, 10% dutycycle
  }

  // Turn off pyros if they have been on for 1 second
  if (!droguePyroOver && droguePyroActive && (micros() - drogueActivationTime >= 1000000)) {
    digitalWrite(PYRO1_DROG, LOW);
    droguePyroOver = true;
    #ifndef FLIGHT
    Serial.println("Drogue pyro deactivated.");
    #endif
  }

  if (!mainPyroOver && mainChutePyroActive && (micros() - mainChuteActivationTime >= 1000000)) {
    digitalWrite(PYRO2_MAIN, LOW);
    mainPyroOver = true;
    #ifndef FLIGHT
    Serial.println("Main chute pyro deactivated.");
    #endif
  }
  */

  ///////////////////////
  ///END STATE MACHINE///
  ///////////////////////

    //Raw Data Stream (to CM4 for logging) - TODO add sample rate esentially

  /*
    if(){
    #ifndef FLIGHT
    String str = String(currentAltitude) + "," + String(movingAvgAccel)  + "," + String(baroVelocity) + "," + 
    String(flightState) + "," + String(latitude) + "," + String(longitude) + "," + String(SIV);

    Serial.println(str);
    #endif
  }
  */

}

