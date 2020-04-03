
/*
 * eBRemote v1.0.6 is the remote unit code for the WAVY electric longboard. It contains code that 
 * reads an analog joystick value and sends that information to the board unit to move or change LEDS.
 * It also contains code to display different data on the LCD screen built into the remote
 * 
 * Note: Turn on the remote first, then turn on the board arduino, other wise will have issues connecting
 * 
 * Author: David Kuehnert
 * 
 * * As of verision 1.0.6:  (04/22/19)
 *  - deleted unneccasary lines of code/print statements
 * 
 * * As of version 1.0.5:  (04/03/19)
 *  - Changed from 8 bit to 16 bit data transfer over radio.
 * 
 * * As of version 1.0.4:
 *  - Radio connects perfectly 
 *  - Communication works between remote and board sending a single byte string back and forth
 *  - LEDs work on both board and remote, and remote can be used to change the light strips
 *  - Both arduinos have knowledge of what is going on with one another thanks to the single byte string
 */
 
//uncomment to see print statements
#define DEBUG


//initation of variables for radio 
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 
RF24 radio(7,8);

// Topology
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };              // Radio pipe addresses for the 2 nodes to communicate.


//initiation of LCD and variable for controlling LCD state
#include <LiquidCrystal.h>              
LiquidCrystal lcd(17,9,15,4,10,14);

enum{SPEED,ODOMETER, ANALOG_VALUES};
volatile int LCD_state = SPEED;
volatile boolean change_state = false;
volatile boolean update_LCD = false;
volatile byte prevPIN;


//initiation of variables for analog stick
volatile int VRx; 
volatile int VRy;
String X;
String Y;

float TRIP = 0;
float initialDIS = 0;
float DIS = 0;
float DIS_int = 0;
float DIS_dec = 0;
float MPH;

bool boardLED = false;
bool switchColors = false;
int time_since_last_colorchange = 0;


volatile bool updateLCD = false;
bool LEDStatusUpdate = false;

int VRx_counter_pos = 0;
int VRx_counter_neg = 0;

enum{High, Medium, Low, Off, Brake};
int speed_state = Off;
int state_timer = 0;



//=============================Setup Functions===========================================

void setup() {
  cli();
  lcd.begin(16,2);
  setupRGB();
  setupAnalogStick();
  setupLightTimers();
  setupLCD_Update_Clock();
  setRGB(0,0,0);
  
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
  setupRadio();                         //sets up Radio setting to send data
  sei();      
}

void setupRadio(){
  printf_begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(0,15);                 // Smallest time between retries, max no. of retries
  radio.setPayloadSize(2);                // Here we are sending 1-byte payloads to test the call-response speed
  radio.openWritingPipe(pipes[0]);        //set up the radio so its sending data
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();                 // Start listening
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
}

/*
 * set up R as port D3, G as port D6, and B as port D5
 */
void setupRGB(){
  DDRD |= 0b01100000;
  PORTD &= 0b10011111;

  DDRC |= 0b0000010;
  PORTC &= 0b11111011;
}

/*
 * sets the Timers for an 8-bit fast PWM output on ports
 * OCOA/B, for control over output value of 
 * the green, and blue brightness. Timer one is currently being used for display purposes
 * and timer 2 is needed to properly control the Tranciever modulo. Because of this the
 * red light is set up to be done as analog
 */
void setupLightTimers(){
  TCCR0A |= 0b10100011; //sets OC0A to normal compare match with mode 3
  TCCR0B |= 0b00000000;
}

/**
 * sets up the ADC4 and ADC5 in order to control the long board. and turns on Pinchange 
 * interupt (PCINT19) for analog stick button (connected to PD2)
 */
void setupAnalogStick(){
  DDRC &= 0b11001111;              //set ADC4/5 as inputs
  DDRD &= 0b11111011;              //set PD2 to input
  PORTD|= 0b00000100;              //enable pull up 

  ADCSRA = 0b10000111;             //enable ADC, 128 clock divider
  ADCSRB = 0;                      //no trigger
  ADMUX = 0b01000100;              //1.1 reference voltage, right justified, ADC4 selected

  cli();
  EIMSK |= 0b00000001;             //external pin interupt for INT0 is enabled
  EICRA = 0;
  EICRA = 0b00001111;             //set to be triggered when button is let go
  EIFR |= 0b00000001;              //clear the flag for INT0
}

/**
 * the LCD can only be written to so quickly, if writing to fast values/ strings wont 
 * display fully. To correct a clock is set to update the screen about 32 times persecond.
 */
void setupLCD_Update_Clock(){
  TCCR2A = 0;
  TCCR2B = 0b00000111;    //set a 64 prescalar
  OCR2A = 255;            //set OCR2A to change update period
  cli();
  TIMSK2 = 0b00000010;
  TIFR2 |= 0b00000010;
}


//================Main Functions=============================================================


void loop() {
  VRx = convertXanalog();
  VRy = convertYanalog();
  if(updateLCD) LCD_State();
  boardLED_status();
  Transmit_Data_to_server();
  DIS = DIS_int + DIS_dec;
}

/*
 * when called will update the LCD display information depending the current state
 * the LCD is in. Update rates are based on OCR1A on Timer 1, and the function is 
 * only called in void loop
 */
void LCD_State(){
  
  if(!LEDStatusUpdate){
    if(change_state){
    change_state = false;
    if(LCD_state == ANALOG_VALUES) LCD_state = SPEED;
    else LCD_state ++;                                           
    }
    switch(LCD_state){
      case SPEED:
        displaySpeed();
        setRGB(0,200,0);
        break;
      case ODOMETER:
        displayOdometer();
        setRGB(0,0,200);
        break;
      case ANALOG_VALUES:
        displayAnalogValues();
        setRGB(200,0,0);
        break;
    }
  }
  else{
    setRGB(0,0,0);
    displayLED_Status();
    setRGB(200,200,200);
    state_timer ++;
    if(state_timer > 100){
      state_timer = 0;
      LEDStatusUpdate = false;
    }
  }
  updateLCD = false;
}


/**
 * takes in 3 values from 0-255 and sets them 
 * to the onboard RGB. Uses timer 0, and 1 with  
 * PWM to control the output. 
 * 
 * note: the red light is being written to with analogWrite
 *       do to minimal timers available for use.
 */
void setRGB(int red, int green, int blue){
  analogWrite(A2, 255-red);
  OCR0A = 255-green;
  OCR0B = 255-blue;
}

/**
 * sets up MUX register to allow for analog conversion
 * of AC0 (x value on the analog) and converts the value
 */
 int convertXanalog(){
    ADMUX = 0b01000100;
    ADCSRA |= 0b01000000;   
    while ((ADCSRA & 0b00010000) == 0);                      //Wait for conversion to complete
    ADCSRA |= 0b00010000;
    return ADC;
 }

 /**
 * sets up MUX register to allow for analog conversion
 * of AC1 (y value on the analog) and converts the value
 */
 int convertYanalog(){
    ADMUX = 0b01000101;
    ADCSRA |= 0b01000000;   
    while ((ADCSRA & 0b00010000) == 0);                     //wait for conversion to complete                        
    ADCSRA |= 0b00010000;
    return ADC;
 }

/*
 * update the LEDs on the Board (toggle on/off, switch colors, nothing) by checking
 * joystick values. If it needs updating will change out going payload to update board.
 */
void boardLED_status(){
   if((VRx_counter_pos || VRx_counter_neg) <= 10){
     if(VRx >= 970) VRx_counter_pos++;
     else if((50 < VRx) && (VRx < 970)){
      VRx_counter_pos = 0;
      VRx_counter_neg = 0;
     }
     else VRx_counter_neg++;  
   }

   if((VRx_counter_pos > 2) && (!LEDStatusUpdate)){
    VRx_counter_pos = 0;
    bool prevBoard = boardLED;
    boardLED = !prevBoard;
    LEDStatusUpdate = true;
   }

   if((VRx_counter_neg > 3) && (time_since_last_colorchange >= 85)){
    VRx_counter_neg = 0;
    if(boardLED){
      time_since_last_colorchange = 0;
      switchColors = true;
      setRGB(0,0,0);
      setRGB(0,200,200);
    }
   }
   time_since_last_colorchange ++;
   if(time_since_last_colorchange > 65536) time_since_last_colorchange = 70;          //rests clock for memory saving (probably uneeded)
}


//============================ Radio Functions =======================================

/*
 * Transmit_Data_to_server when called will send out data to the board unit if connected. It then
 * takes in the acknowledged data from the Board and decrypts it
 */
 void Transmit_Data_to_server(){
    radio.stopListening();                                  // First, stop listening so we can talk.
    
    uint16_t payload;
    uint16_t recievedLoad;
    payload = encryptPayload();   
                                                            //Called when STANDBY-I mode is engaged (User is finished sending)
    if (!radio.write( &payload, 2 )){
      Serial.println(F("failed."));      
    }else{

      if(!radio.available()){ 
        Serial.println(F("Blank Payload Received.")); 
      }else{
        while(radio.available() ){
          radio.read( &recievedLoad, 2 );
          FloatUnrounder(recievedLoad); 
        }
      }
     
    }
    delay(1);
 }


/*
 * FloatUnrounder takes the acknowledge payload from the board and updates the current speed/odom
 * for the LCD to display
 */
float FloatUnrounder(uint16_t b)
{
  if(b & 0x0001){                                         //Checks to see if byte string is MPH
    uint16_t mph_int = b & 0xFC00;
    uint16_t mph_dec = b & 0x03C0;
    int MPH_int = (mph_int >> 10);
    float MPH_dec = (mph_dec >> 6)/10.00;
    
    MPH = MPH_int + MPH_dec;
  }
  else{
    if(b & 0x8000){                                      //Checks to see if byte string is DIS dec
      uint16_t dis_dec = b & 0x7FFE;
      DIS_dec = (dis_dec>>1)/10000.00;
    }
    else{
      uint16_t dis_int = b & 0x7FFE;
      DIS_int = (dis_int>>1)*1.00;
    }
    if((DIS_dec + DIS_int > 0) && (initialDIS == 0)){
      initialDIS = DIS_int + DIS_dec;
    }
    TRIP = DIS - initialDIS;
  }
}

/*
 * encryptPayload() is a helper function that takes all the current data the remote has, and turns
 * it into a 16 bit string for the radio to send to the board.
 */
uint16_t encryptPayload(){
  uint16_t out = 0;

  //encrypt y analog value into the byte string
  uint16_t analogY = VRy;
  out = (analogY << 5);

  //encrypt LED status to byte
  if(boardLED) out |= 0x0008;                   //Set LED to be On
  if(switchColors){
    switchColors = false;
    out |= 0x0018;                              //Set LED to change color
  }
  out |= 0x8000;                                //Set flag to let it be known its data
  return out;
}

//============================= LCD Print Functions =============================

/**
 * displays the current speed the board is traveling 
 * Will display
 * 
 * XX.X mph
 */
void displaySpeed(){
  lcd.clear();
  String str_speed = String(MPH);
  char charX[10];
  str_speed.toCharArray(charX,10);
  lcd.write("SPEED: ");
  lcd.setCursor(0,1);
  lcd.write(charX);
}

/*
 * displays current travel distance since last power up, and the total distance 
 * traveled on the board.
 * Will display:
 * Current: XX.X mi
 * Overall: XXX.X mi
 */
void displayOdometer(){
  lcd.clear();
  String str_trip = String(TRIP);
  char char1[10];
  String str_distance = String(DIS);
  char char2[10];

  str_trip.toCharArray(char1,10);
  str_distance.toCharArray(char2,10);
  lcd.write("TRIP: ");
  lcd.setCursor(6,0);
  lcd.write(char1);

  lcd.setCursor(0,1);
  lcd.write("ODOM: ");
  lcd.setCursor(6,0);
  lcd.write(char2);
  
}

/**
 * displays up to date readings from the analog joystick for use of debugging
 * will display:
 * X: (value 0-1023)
 * Y: (value 0-1023=)
 */
void displayAnalogValues(){
  lcd.clear();
  String strx = String(VRx);
  String stry = String(VRy);
  char charX[10];
  char charY[10];
  strx.toCharArray(charX,10);
  stry.toCharArray(charY,10);
  
  lcd.setCursor(0,0);
  lcd.write("X: ");
  lcd.setCursor(3,0);
  lcd.write(charX);
  lcd.setCursor(0,1);
  lcd.write("Y: ");
  lcd.setCursor(3,1);
  lcd.write(charY);
  delay(10);
}   

/*
 * displayLED_Status will print on the LED if the LED's have just been turned on
 * or if they have just turned off. Only called when joystick is swiped to the right
 */
void displayLED_Status(){
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.write("Turning LED's");
    lcd.setCursor(7,1);
    if(boardLED)lcd.write("ON");
    else lcd.write("OFF");
}


//============================= ISR's ===========================================

/*
 * external interrupt on pin 2 which is connected to joystick. If pressed, will change
 * the state of the LCD.
 */
ISR(INT0_vect){
  cli();
  change_state = true;
  EIFR |= 0b00000001;
  sei();
}

/*
 * Timer 2 compa vect is an interupt that turns a boolean on when its time to update the LCD.
 * This keeps the LCD from looking glitchy do to update rates being to high
 */
ISR(TIMER2_COMPA_vect){
  cli();
  updateLCD = true;;
  sei();
}
