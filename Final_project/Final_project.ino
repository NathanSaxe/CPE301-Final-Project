#include <LiquidCrystal.h>
#include <DHT11.h>
#include <Stepper.h>
#include <RTClib.h>
#include <string.h>

#define RDA 0x80
#define TBE 0x20
int state = 0;

// DHT Objects
DHT11 DHT(10); //Pin 10
RTC_DS3231 rtc;

//STEPPER MOTOR PRESETS
const int stepsPerRevolution = 2038;
Stepper Vent = Stepper(stepsPerRevolution, 22, 24, 26 , 28);
int ventPosition = 45;


//WATER COOLER PRESETS
const int WATERSENSORPIN = 0;
const int STARTBUTTON = 13;
volatile bool coolerOn = false;
// A Pointers(Digital Pins 22-29)
// Primarily in charge of button input
//Button is at Pin 29

volatile unsigned char *PIN_A = (unsigned char *)0x20;
volatile unsigned char *DDR_A = (unsigned char *)0x21;
volatile unsigned char *PORT_A = (unsigned char *)0x22;
// B Pointers(Digital Pins 10-13, 50-53)
//Receives serial data from DHT11 at Pin 10/PB4

volatile unsigned char *PIN_B = (unsigned char *)0x23;
volatile unsigned char *DDR_B = (unsigned char *)0x24;
volatile unsigned char *PORT_B = (unsigned char *)0x25;


// C Pointers(Digital Pins 30-37)
// Outputs LED light

//Red light is at Pin 35
//Green light is at Pin 33
//Blue light is at Pin 31
volatile unsigned char *PIN_C = (unsigned char *)0x26;
volatile unsigned char *DDR_C = (unsigned char *)0x27;
volatile unsigned char *PORT_C = (unsigned char *)0x28;


// D Pointers(Digital Pins 38)

volatile unsigned char *PIN_D = (unsigned char *)0x29;
volatile unsigned char *DDR_D = (unsigned char *)0x2A;
volatile unsigned char *PORT_D = (unsigned char *)0x2B;

// F Pointers(Analog Pins 0-7)
//PF0 holds the Temperature/Humidity sensor
//PF7 holds the water sensor
volatile unsigned char *PIN_F = (unsigned char *)0x2F;
volatile unsigned char *DDR_F = (unsigned char *)0x30;
volatile unsigned char *PORT_F = (unsigned char *)0x31;

// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
// ADC Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char* MOTOR_DDR = (unsigned char*) 0x10A; // PORT L PINS 42 - 49 inclusive
volatile unsigned char* MOTOR_PORT = (unsigned char*) 0x10B; // PORT L PINS 42 - 49 inclusive

int dir1 = 4;
int dir2 = 3;

//LCD SCREEN PRESETS
//LCD Pins to Arduino Pins
const int RS = 12, EN = 11, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


void setup() {
  U0Init(9600);
  lcd.begin(16,2);
  rtc.begin();
  lcd.clear();
  createLCDReadout();
  attachInterrupt(digitalPinToInterrupt(STARTBUTTON), blink, RISING);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  adc_init(); //start ADC module
}

void loop() {
  // put your main code here, to run repeatedly:

  switch(state){
    case 0:

    case 1:

    case 2:

    case 3:
  }
}

/*
 * UART FUNCTIONS
 */
void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40; //0b 0100 0000
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char getChar()
{
  return *myUDR0;
}

void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void moveVent(int direction){
  if (direction == -1 && ventPosition > 0){
    Vent.step(-stepsPerRevolution/48); //7.5* shift
    ventPosition -= 7.5;
  } else if (ventPosition < 90 && direction == 1){
    Vent.step(stepsPerRevolution/48);
    ventPosition += 7.5;
  }
}

int getTemp(){
  int temperature = DHT.readTemperature();
  return temperature;
}

//check humidity
int getHumidity(){
  int humidity = DHT.readHumidity();
  return humidity;
}

int checkWaterLevel(){
  int waterlevel = adc_read(WATERSENSORPIN);
  if(waterlevel < 200){

  }
}

void getTime(){
  DateTime now = rtc.now();
}

void createLCDReadout(){
  lcd.print("Temperature:  *C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity:   %");
}

void blink(){
  state = !state;
}

void setGPIOPins(){
  //Set A registers
  //Set PA7 to input with pullup
  *DDR_A |= 0x00; //0b 0000 0000
  *PORT_A |= 0x70; //0b 1000 0000

  //Set B registers
  //PB4 inputs serial data from DHT11
  *DDR_B |= 0x00; //0b 0000 0000
  *PORT_B |= 0x04; //0b 0000 1000
  
  //Set C Registers
  //Set Lights to receive output
  //Red is at PC2
  //Green is at PC4
  //Blue is at PC6
  *DDR_C |= 0x54; //0b 0101 0100
  *PORT_C |= 0x00; //0b 0000 0000

  //Set D Registers

  *DDR_D |= 0x00;
  *PORT_D |= 0x00;

  //Set F Registers
  //PF7 holds the water sensor

  *DDR_F |= 0x00;
  *PORT_F |= 0x70;
}