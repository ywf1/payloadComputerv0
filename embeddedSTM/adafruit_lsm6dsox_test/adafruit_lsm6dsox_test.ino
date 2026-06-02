
#include <Adafruit_LSM6DSOX.h>
#include <SPI.h>

// For SPI mode, we need a CS pin
#define LSM_CLK PA5
#define LSM_MISO PA6
#define LSM_MOSI PA7
#define LSM_CS PB0
#define LSM_INT2 PB1
#define LSM_INT1 PB10

#define UART_TX_RW_RX PA9
#define UART_RX_RW_TX PA10

HardwareSerial debugSerial(UART_RX_RW_TX,UART_TX_RW_RX);
//            MOSI  MISO  SCLK   SSEL
SPIClass mySPI(LSM_MOSI,LSM_MISO,LSM_CLK);

Adafruit_LSM6DSOX sox;
void setup(void) {
  debugSerial.begin(115200);
  while (!debugSerial)
    delay(10); // will pause Zero, Leonardo, etc until debugSerial console opens

  debugSerial.println("Adafruit LSM6DSOX test!");

  if(!sox.begin_SPI(LSM_CS,&mySPI)){
    debugSerial.println("LSM6DSOX NOT Found!");
    while(1){};
  }

  debugSerial.println("LSM6DSOX Found!");

  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  debugSerial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    debugSerial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    debugSerial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    debugSerial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    debugSerial.println("+-16G");
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  debugSerial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    debugSerial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    debugSerial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    debugSerial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    debugSerial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    debugSerial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  debugSerial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    debugSerial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    debugSerial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    debugSerial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    debugSerial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    debugSerial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    debugSerial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    debugSerial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    debugSerial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    debugSerial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    debugSerial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    debugSerial.println("6.66 KHz");
    break;
  }

  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  debugSerial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    debugSerial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    debugSerial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    debugSerial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    debugSerial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    debugSerial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    debugSerial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    debugSerial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    debugSerial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    debugSerial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    debugSerial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    debugSerial.println("6.66 KHz");
    break;
  }
}

void loop() {

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  debugSerial.print("\t\tTemperature ");
  debugSerial.print(temp.temperature);
  debugSerial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  debugSerial.print("\t\tAccel X: ");
  debugSerial.print(accel.acceleration.x);
  debugSerial.print(" \tY: ");
  debugSerial.print(accel.acceleration.y);
  debugSerial.print(" \tZ: ");
  debugSerial.print(accel.acceleration.z);
  debugSerial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  debugSerial.print("\t\tGyro X: ");
  debugSerial.print(gyro.gyro.x);
  debugSerial.print(" \tY: ");
  debugSerial.print(gyro.gyro.y);
  debugSerial.print(" \tZ: ");
  debugSerial.print(gyro.gyro.z);
  debugSerial.println(" radians/s ");
  debugSerial.println();

  delay(100);

  //  // debugSerial plotter friendly format

  //  debugSerial.print(temp.temperature);
  //  debugSerial.print(",");

  //  debugSerial.print(accel.acceleration.x);
  //  debugSerial.print(","); debugSerial.print(accel.acceleration.y);
  //  debugSerial.print(","); debugSerial.print(accel.acceleration.z);
  //  debugSerial.print(",");

  // debugSerial.print(gyro.gyro.x);
  // debugSerial.print(","); debugSerial.print(gyro.gyro.y);
  // debugSerial.print(","); debugSerial.print(gyro.gyro.z);
  // debugSerial.println();
  //  delayMicroseconds(10000);
}