//adc test payload comp
#define LIGHT PA4
#define CONT PA1

#define UART_TX_RW_RX PA9
#define UART_RX_RW_TX PA10

HardwareSerial debugSerial(UART_RX_RW_TX,UART_TX_RW_RX);

void setup() {
  delay(1000);
  // put your setup code here, to run once:
  pinMode(LIGHT,INPUT_ANALOG);
  pinMode(CONT,INPUT_ANALOG);

  debugSerial.begin(115200);
  
  debugSerial.println("ADC Example begin!");
}

void loop() {
  // put your main code here, to run repeatedly:
  debugSerial.println(analogRead(CONT));
  //debugSerial.println(analogRead(LIGHT));
  delay(500);
}
