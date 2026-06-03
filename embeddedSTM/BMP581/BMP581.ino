#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

#define BAR_INT PA15
#define BAR_SCL PB6
#define BAR_SDA PB7

// Create a new sensor object
BMP581 pressureSensor;

// I2C address selection
//uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
uint8_t i2cAddress = BMP581_I2C_ADDRESS_SECONDARY; // 0x46

TwoWire barWire(PB_7, PB_6);

#define UART_TX_RW_RX PA9
#define UART_RX_RW_TX PA10

HardwareSerial debugSerial(UART_RX_RW_TX,UART_TX_RW_RX);

void setup()
{
    delay(1000);
    // Start debugSerial
    debugSerial.begin(115200);
    debugSerial.println("BMP581 Example1 begin!");

    // Initialize the I2C library
    barWire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x47)
    while(pressureSensor.beginI2C(i2cAddress,barWire) != BMP5_OK)
    {
        // Not connected, inform user
        debugSerial.println("Error: BMP581 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    debugSerial.println("BMP581 connected!");
}

void loop()
{
    // Get measurements from the sensor
    bmp5_sensor_data data = {0,0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if(err == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        debugSerial.print("Temperature (C): ");
        debugSerial.print(data.temperature);
        debugSerial.print("\t\t");
        debugSerial.print("Pressure (Pa): ");
        debugSerial.println(data.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        debugSerial.print("Error getting data from sensor! Error code: ");
        debugSerial.println(err);
    }

    // Only print every second
    delay(1000);
}