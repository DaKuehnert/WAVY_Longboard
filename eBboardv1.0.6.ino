
/*
 * eBboard v1.0.4 is the on board unit code for the WAVY electric longboard. It contains code that 
 * allows driving, radio communication, speed collection, and displaying of the LED light strips.
 * 
 * Note: Turn on the remote first, then turn on the board arduino, other wise will have issues connecting
 * 
 * Author: David Kuehnert
 * 
 * * As of verision 1.0.6:  (04/22/19)
 *  - Added EEPROM
 *  - Fixed distance calculations for GPS
 * 
 * * As of version 1.0.5:  (04/03/19)
 *  - Changed from 8 bit to 16 bit data transfer over radio. 
 *  - Added tested code for GPS
 * 
 * * As of version 1.0.4:
 *  - Radio connects perfectly 
 *  - Communication works between remote and board sending a single byte string back and forth
 *  - LEDs work on both board and remote, and remote can be used to change the light strips
 *  - Both arduinos have knowledge of what is going on with one another thanks to the single byte string
 */
 
//uncomment to see print statements
#define DEBUG



//setup for gps
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//initation of radio library
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"



//setup for radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 
RF24 radio(7,8);

// Topology
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };              // Radio pipe addresses for the 2 nodes to communicate.

// A single byte to keep track of the data being sent back and forth
uint16_t payload;
uint16_t sendOut;



//Setup for data output
uint16_t MPHb;
uint16_t DIS_int;
uint16_t DIS_dec;

enum{mph_out, dis_int_out, dis_dec_out};
int send_out_state = mph_out;
bool switch_state = false;

int analogY;




//setup for LEDS
enum{LED_OFF, LED_SOLID};
int LED_state = LED_OFF;

enum{RED, YELLOW, GREEN, BLUE, VIOLET, WHITE};
int solidcolor_state = YELLOW;
bool solidcolor_switch = false;

//Define RGB values of all the colors  (the led strip is broken so orange and indigo dont show);
int red[] = {255,0,0};
//int orange[] = {225,135,0};
int yellow[] = {250,255,0};
int green[] = {0,255,0};
int blue[] = {0,0,255};
//int indigo[] = {255,0,100};
int violet[] = {148,0,211};
int white[] = {255,255,254};
int ledoff[] = {0,0,0};



//drive motor setup
#include <Servo.h>
Servo motor1;


//other setup
float DIS = 0.0;
float MPH = 12.9;

float prevDIS = 0.0;

bool allowDrive = false;
int disconectTimer = 0;
int state_timer = 0;

float speedStartStop[] = {0.0,0.0};
long timer = 0;

//=============================Setup Functions===========================================

void setup() 
{
  setupLights();
  
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
  setupRadio();
  
  //setupMotors();
  
  motor1.attach(PD5);
  
  ss.begin(GPSBaud);

  readEEPROM();                                             //Read EEPROM to get old distance value
  
}

/**
 * setupMotors is currently never called but was originally for controlling the ESCs. For somereason
 * it wasn't working properly even trying it on Timer 0 vs Timer 1. Currently the code uses the Servo 
 * library for writing to motors
 */
void setupMotors(){
//  DDRD |= 0b01100000;
//  PORTD &= 0b10011111;
//
//  TCCR0A = 0;
//  TCCR0B = 0;
//  TCCR0A |= 0b10100011;                  //sets OC0A to normal compare match with mode 3

//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCCR1A |= 0b10100011;                  //Set timer to non inverting for A/B and 8 bit fast PWM
//  TCCR1B |= 0b00011001;                  //Set timer to non inverting 8 bit fast PWM
//  OCR1A = 39999;
}

/**
 * setupRadio() configures the setting of the nRF24 radio modulo to be able to talk to the other 
 * arudino radio.
 */
void setupRadio(){
  printf_begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(0,15);                 // Smallest time between retries, max no. of retries
  radio.setPayloadSize(2);                // Here we are sending 1-byte payloads to test the call-response speed
  radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, and switch when writing
  radio.openReadingPipe(1,pipes[0]);      //sets the radio so its listening for data
  radio.startListening();                 // Start listening
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
}

/**
 * setupLights sets the A0, A1, and A2 as outputs for an analog write to control LED strip. It then
 * turns off the lights
 */
void setupLights(){
  DDRC |= 0b00000111;
  setLED(ledoff);
}

//======================================== Main Functions ====================================================

/*
 * loop() is the main function of the code. It loops through collecting MPH data, and Distance data from
 * the gps, and sends it out to the remote. It then collects data from the remote, and moves the board
 */
void loop()
{ 
  distanceCalculation(0);
  FloatRounderMPH(MPH);
  FloatRounderDIS(DIS);
  recieve_data_from_slave();
  boardLED();
  moveBoard(analogY);
  if(gps.speed.isValid()){
    MPH = gps.speed.mph();
  }
  distanceCalculation(1);
}

/*
 * boardLED() is a state machine that determines if the LED strip should be on or off
 */
void boardLED(){
  switch(LED_state){
    case LED_OFF:
      setLED(ledoff);
      break;
    case LED_SOLID:
      setSolidColor();
      break;
  }
}

/*
 * setLED takes in a color that has been predefined to contain all three RGB values.
 * Since this is a common annode RGB, the analogWrite acts as ground so its 255 - the desired
 * R, G, or B value.
 * note: B and G pin are flipped on these LEDS
 */
void setLED(int color[]){
    analogWrite(A0, 255 - color[0]);
    analogWrite(A1, 255 - color[1]);
    analogWrite(A2, 255 - color[2]);
}

/*
 * setSolidColor() is a helper switch case function that contains all colors of the LEDs the user
 * can flip through.
 */
void setSolidColor(){
  switch(solidcolor_state){
        case RED:
          setLED(red);
          break;
//        case ORANGE:
//          setLED(orange);
//          break;
        case YELLOW:
          setLED(yellow);
          break;
        case GREEN:
          setLED(green);
          break;
        case BLUE:
          setLED(blue);
          break;
//        case INDIGO:
//          setLED(indigo);
//          break;
        case VIOLET:
          setLED(violet);
          break;
        case WHITE:
          setLED(white);
          break;
      }
}

/*
 * moveBoard takes in an analogValue taken in via radio, and is written to using the servo library, iff
 * the remote has been in connection
 */
void moveBoard(int joyStick){
   if(allowDrive){                                            //setsup a saftey net just incase remote disconnects the board will stop
      int pulseWidth = map(joyStick, 0, 1020, 0, 180);
      Serial.println(pulseWidth);
      motor1.write(pulseWidth);
    }
    else{
      motor1.write(0);
    }
}

/*
 * distanceCalculation samples data from the gps to determine the total distance the board has traveled.
 * If the perameter is 0 its the first sample and if its 1 then you take the second sample
 */
void distanceCalculation(int pos){
  if(gps.speed.isValid()){
    if(pos = 0){
      speedStartStop[0] = gps.speed.kmph();
      timer = millis();
    }
    else{
      speedStartStop[1] = gps.speed.kmph();
      long end_timer = millis();
      DIS += (speedStartStop[1] - speedStartStop[0])/((float)(end_timer - timer)); 
      speedStartStop[0] = 0.0;
      speedStartStop[1] = 0.0; 
    }
  }
}

//============================================ Radio Functions =====================================================

/*
 * revieve_data_from_slave() is a function that when calls, checks to see if the other radio is connected and
 * sending out data. If it is, it decrypts the data and sends out the respective return package. Other wise it 
 * starts to keep track of how long it has been since the last time the board heard from the remote
 */
void recieve_data_from_slave(){
  byte pipeNo; 
  sendOut = getsendOut();
  if(!radio.available(&pipeNo)){
    disconectTimer ++;
    if(disconectTimer > 80){
      allowDrive = false;
        disconectTimer =0;
    }
  }
  while(radio.available(&pipeNo)){
    radio.read( &payload, 2 );
    radio.writeAckPayload(pipeNo,&sendOut, 2);
    allowDrive = true;
    if(payload == 0 ) allowDrive = false;  
  }
  decryptPayload(payload);
}

/**
 * getsendOut() is a helper function that rotates through the data that is ment to be sent back to 
 * the remote
 */
uint16_t getsendOut(){
  switch(send_out_state){
    case mph_out:
      send_out_state = dis_int_out;
      return MPHb;
      break;
    case dis_int_out:
      send_out_state = dis_dec_out;
      return DIS_int;
      break;
    case dis_dec_out:
      send_out_state = mph_out;
      return DIS_dec;
      break;
  }
}

/*
 * decryptPayload takes a 16 bit string and decodes it into usable data for the board unit arduino
 */
void decryptPayload(uint16_t b){
  if(b & 0x8000){
    analogY = (b & 0x7FE0) >> 5;
    byte LED = (b & 0x0018) >> 3;
                     
    if(LED == (0b00000001)) LED_state = LED_SOLID;
    else if(LED == (0b00000011) && state_timer >= 60) {
      Serial.println(" Changing Colors");
      state_timer = 0;
      if(solidcolor_state == WHITE) solidcolor_state = RED;
      else solidcolor_state ++;  
    }
    else LED_state = LED_OFF;
  }
  state_timer ++; 
}

//====================================== Rounder Functions ===============================================

/*
 * FloatRounderMPH takes in a float value of the speed of the board,rounds it to the nearest tenths
 * place, and it turns it into a 16 bit string that can be recieved by the other arduino.
 */
void FloatRounderMPH(float f){
  if (f < 64){
    MPHb = 0x0000;
    //byte int_floored = 0;
    uint16_t int_floored =  floor(f);
    uint16_t intReturn = (int_floored << 10);
    
    double dec = (f-floor(f))* 10;
    byte dec_floored = (byte) round(dec);
    uint16_t decReturn = (dec_floored<<6);
  
    MPHb |= intReturn;
    MPHb |= decReturn;
    MPHb |= 0x0001;
  }
}

/*
 * FloatRounderMPH takes in a float value of the Distance the board has traveled, rounds it to the 
 * nearest tenthousandths place, and it turns it into a 16 bit string that can be recieved by the 
 * other arduino.
 */
void FloatRounderDIS(float f){
  if (DIS < 150000){
    DIS_int = 0x0000;
    uint16_t int_floored =  floor(f);
    DIS_int = (int_floored << 1);
    
    DIS_dec = 0x0000;
    double dec = (f-floor(f)) * 10000;
    uint16_t dec_floored = round(dec);
    DIS_dec = (dec_floored << 1);
    DIS_dec |= 0x8000;                                  //set flag to let remote know its the decimal of MPH 
    
  } 
  else{
    //reset the odometer
    DIS = 0;
    DIS_int = 0x0000;
    DIS_dec = 0x0000;
  } 
}

//===================================== EPROM Functions =============================================
void updateEEPROM(){
  if(DIS - prevDIS > 0.1){                                                  //only update EEPROM if distance has changed
    byte DIS_intH = (DIS_int >> 8);
    byte DIS_intL = DIS_int & (0x00FF);
  
    byte DIS_decH = (DIS_dec >> 8);
    byte DIS_decL = DIS_dec & (0x00FF);
  
    byte DataWrite[] = {DIS_intH,DIS_intL,DIS_decH,DIS_decL};
  
    for(int i = 0; i<4; i++){
      EEPROM_write(i, DataWrite[i]);
    }
    prevDIS = DIS;
  }
}

void readEEPROM(){
  byte DataWrite[] = {0x00, 0x00, 0x00, 0x00};
  for(int i= 0; i<4; i++){
    DataWrite[i] = EEPROM_read(i);
  }
  
  uint16_t decval = ((DataWrite[2] << 8) || DataWrite[3]) & 0x7FFE;
  DIS_dec = (decval>>1)/10000.00;
 
  uint16_t intval = ((DataWrite[0] << 8) || DataWrite[1]) & 0x7FFE;
  DIS_int = (intval>>1)*1.00;
  
  DIS = DIS_int + DIS_dec;
}

void EEPROM_write(unsigned int Address, byte Data){
  while (EECR & 0x02){}                                 //Wait for completion of previous write

  EEAR = Address;                                       //set up address register
  EEDR = Data;                                          //Set up data register

  cli();
  EECR |= 0b00000100;
  EECR |= 0b00000010;                                   //start EEPROM write
  sei();
}

byte EEPROM_read(unsigned int Address){
  while (EECR & 0x02){}                                 //Wait for completion of previous write

  EEAR = Address;
  EECR |= 0b00000001;
  return EEDR;
}
  
