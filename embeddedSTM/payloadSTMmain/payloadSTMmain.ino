#include "STM32pins.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>

#define Serial piSerial
#define numSamples 30 //# of values in moving average
#define BUFF_SIZE 20

//LSM6DSOX SPI
SPIClass imuSPI(IMU_MOSI,IMU_MISO,IMU_CLK);



//RPI UART
//HardwareSerial piSerial(RPI_TX_STM_RX,RPI_RX_STM_TX);
//baro wire
TwoWire barWire(PB_7, PB_6);

//RW/debug UART
//HardwareSerial rcSerial(RX_1_RC_TX,TX_1_RC_RX);


//Peripheral Object Decleration
Adafruit_LSM6DSOX lsm6dsox;                    // LSM6DSOX IMU sensor object
BMP581 pressureSensor;

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
unsigned long lastLightTime = 0;

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
  pinMode(PI_EN,OUTPUT); pinMode(PYRO,OUTPUT); pinMode(LED,OUTPUT);


  digitalWrite(LED,HIGH); digitalWrite(PI_EN,LOW); digitalWrite(PYRO,LOW);

  //Serial Initalization
  piSerial.begin(115200);  
  rwSerial.begin(115200);

  /*
  IMU SETUP
  */

  // Initialize LSM6DSOX IMU over SPI
  if (!lsm6dsox.begin_SPI(imuSPI,&SPI_4)) {
    //Serial.println("LSM6DSOX not detected. Check wiring.");
  }
  //accel setup
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G); // Set Acceleration Range to max (16G)
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ); //set Accel Data Rate
  lsm6dsox.configInt2(false,false,true); //Configure Innterupt when Accel Data Ready
  attachInterrupt(digitalPinToInterrupt(IMU_INT2), accelIRQ, RISING); // set interrupt pin for lsm6dsox acceleration data
  
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  lsm6dsox.configInt1(false,true,false);
  attachInterrupt(digitalPinToInterrupt(IMU_INT1), gyroIRQ, RISING); // set interrupt pin for lsm6dsox acceleration data

  /*
  BAROMETER SETUP
  */

  barWire.begin();
  // Check if sensor is connected and initialize
  pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY)
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
  attachInterrupt(digitalPinToInterrupt(BMP_INT), baroIRQ, RISING);
  
  ////////////
  ////CONT////
  ////////////

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
    int rawValue = analogRead(LIGHT);
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
    }

    //Calculate moving average and flip if necessary
    if (accelBaselineSet) {
      movingAvgAccel = accelSum / numSamples;

      if(movingAvgAccel > 0 && (!imuFlipped)){
        imuFlipped = true;
        imuFlip = -1.0;
      } 
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
    totalXrot += gyro.gyro.x * gyroDt;
    totalYrot = gyro.gyro.y * gyroDt;
    totalZrot = gyro.gyro.z * gyroDt;
  }

  //light sensor updating:
  //checking the light level on the photo resistor
  if (currentTime - lastLightTime >= lightSampleInterval) {
    int rawValue = analogRead(LIGHT_SENSER);
    float voltage = (rawValue / ADC_RESOLUTION) * REF_VOLTAGE;

    // Update moving average
    voltageSum -= voltageBuffer[bufferIndex];
    voltageBuffer[bufferIndex] = voltage;
    voltageSum += voltage;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    if (bufferCount < BUFFER_SIZE)
      bufferCount++;

    averageVoltage = voltageSum / bufferCount;
  }

  if(!liftoffDetected){
    baroWeight = 0.1;
    accelWeight = 0.0;
  }else if(liftoffDetected && filteredVelocity <= 600 && movingAvgAccel >= -6.0){
    baroWeight = 0.3;
    accelWeight = 0.7;
  }else if(liftoffDetected && filteredVelocity > 600 && movingAvgAccel >= -6.0){
    baroWeight = 0.0;
    accelWeight = 1;
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
    if ((currentAltitude >= liftoffAltitudeThreshold) || (movingAvgAccel >= (liftoffAccelThreshold - 9.8) && currentAltitude >= 10)){
      liftoffDetected = true;
      digitalWrite(LED, LOW);  // Turn on LED for liftoff indication
      digitalWrite(PI_EN,HIGH);
      
      logData = true;
      liftoffTime = micros();
      flightState = 2;

    }
  }

  // nosecone deployment sensed
  if (liftoffDetected && averageVoltage >= basevoltageread + LIGHT_THRESHOLD) {
    noseOff = true;
    noseOffTime = micros();
    flightState = 5;
  }

  // nosecone deployment sensed
  if (liftoffDetected && noseOff && currentTime - noseOffTime >= 2000000 && !tenderCut) {
    digitalWrite(PYRO,HIGH);
    tenderCut = true;
    tenderCutTime = micros();
    flightState = 6;
  }

    // nosecone deployment sensed
  if (liftoffDetected && noseOff && tenderCut && currentTime - tenderCutTime >= 1000000) {
    digitalWrite(PYRO, LOW);
    //Enable active stabilization
    digitalWrite(RW_EN, HIGH);
    rwEnabled = true;
    flightState = 7;
  }

  //landing detection
  if (tenderCut && baroVelocity >= -1) {
    flightState = 8;
    landed = true;
    //disable RW
    digitalWrite(RW_EN, LOW);
    //indicate landing has been detected
    digitalWrite(LED,LOW);
    //PI shutdown?
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

