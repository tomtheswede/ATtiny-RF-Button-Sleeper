/*  
 *   For sending 433MHz RF signals from battery operated ATtiny85
 *   Code by Thomas Friberg (https://github.com/tomtheswede)
 *   Updated 27/05/2017
 */

#include <avr/sleep.h> //For sleep commands set_sleep_mode(SLEEP_MODE_PWR_DOWN), sleep_enable() and sleep_mode();

//Device parameters
const unsigned long devID = {5123434355,1656346333}; // 00001010111011011000100111101111 So the message can be picked up by the right receiver
const unsigned long devType = {31,34}; //Reads as "1" corresponding with BTN type

//General variables
const byte pwrPin = 1; //Power for external elements pin. pin 6 for rfbutton v0.7 pin 6 is PB1 so use 1

//RF related
const byte sendPin = 4; //RF pin. pin 3 for rfbutton v0.7 which is pb4 so this variable should be 4.
const byte typePreamble[4] = {120,121,122,123}; //01111000 for register, 01111001 for basic message type
byte msgLengths[4]={32,8,16,32};
byte msgBuffer[9]; //Max 9 bytes for transmission
int msgLength;
int repQuant;

//Button related
unsigned long currentTime=millis();
unsigned long lastBatRead=0;
boolean postRelease=false;
unsigned int timeOutCount=0;
unsigned long releaseTime=millis();

//Interrupt variables
volatile const byte btnPin = 0; //rfButton v0.4 uses pin 5 which is PB0 so use 0 here.
volatile bool primer[5]={0,0,0,0,0};
volatile bool pressed=0;
volatile bool buttonState=0;
volatile unsigned long pressTime=0;
volatile const unsigned int reTriggerDelay=40; //minimum time in millis between button presses to remove bad contact
volatile unsigned long watchdogCounter=0;
volatile bool batteryFlag=false;
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) //OR - Turn on bit
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) //AND - Turn off bit

void setup() { //This code runs as soon as the device is powered on.
  pinMode(sendPin,OUTPUT);
  pinMode(pwrPin,OUTPUT);
  digitalWrite(pwrPin,1);
  pinMode(btnPin,INPUT_PULLUP);

  encodeMessage(0,devType[0]); //Register on first on for button
  delay(400);
  encodeMessage(0,devType[1]); //Register on first on for battery
  delay(400);
  encodeMessage(2,getVCC()); //Report battery on first on
  delay(10);

  digitalWrite(pwrPin,0); //Power down devices between button pushes
  WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0); //Sets the watchdog to ~8 seconds
  WDTCR |= (1<<WDIE); //Enables the watchdog interupt
  //Set pin to start interrupts
  sbi(GIMSK,PCIE); //Turn on interrupt
  sbi(PCMSK,PCINT0); //set pin affected by interupt - PCINT0 corresponds to PB0 or pin 5
  sleepSet(); //Sleep when entering loop
}

//Main Loop --------------------

void loop() {
  CheckButton();
}

//System functions--------------

void sleepSet() {
  digitalWrite(pwrPin,0); //Turn off ancillaries
  //Power down
  cbi(ADCSRA,ADEN); //Turns off the ADCs
  sei(); //Enables all interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode(); //Starts the sleep
  //Sleep is here. Code only runs onward if an intterupt is triggered.
  sbi(ADCSRA,ADEN); //Turns on the ADCs when leaving sleep
  //cli(); //Turn off all interrupts
}

int getVCC() { //Returns battery voltage in millivolts
  //From https://github.com/cano64/ArduinoSystemStatus/blob/master/SystemStatus.cpp
  ADMUX = _BV(MUX3) | _BV(MUX2); // For ATtiny85/45
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low = ADCL;
  unsigned int val = (ADCH << 8) | low;
  //discard previous result
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  low = ADCL;
  val = (ADCH << 8) | low;
  lastBatRead=millis();
  return ((long)1024 * 1100) / val;
}

//Button functions --------------------------------------------

ISR(PCINT0_vect) {
  // This is called when the interrupt occurs
  buttonState=!(digitalRead(btnPin));
  if (buttonState && (millis()-pressTime>reTriggerDelay)) { //if pressed in   // && (millis()-releaseTime>reTriggerDelay)
    pressed=true;
    primer[0]=1;
    primer[1]=1;
    primer[2]=1;
    primer[3]=1;
    primer[4]=1;
    pressTime=millis();
  }
}

ISR(WDT_vect) { //Only triggers when button is not pressed due to interupts being disabled when not in sleep
  watchdogCounter++;
  if (watchdogCounter>3115) { //The number here determines frequency of report. 30 is equivalent to 208 seconds. 3115 is equivalent to 6hrs
    batteryFlag=true;
  }
  else {
    sleepSet(); //Sleep when button is released and measurements complete
  }
}

void CheckButton() {
  if (pressed) {
    digitalWrite(pwrPin,1); //Turn on ancillaries
    currentTime=millis();
    if (primer[0]) {
      encodeMessage(1,1);
      primer[0]=0;
    }
    else if (primer[1] && (currentTime-pressTime>600)) {
      encodeMessage(1,2);
      primer[1]=0;
    }
    else if (primer[2] && (currentTime-pressTime>1500)) {
      encodeMessage(1,3);
      primer[2]=0;
    }
    else if (primer[3] && (currentTime-pressTime>3000)) {
      encodeMessage(1,4);
      primer[3]=0;
    }
    else if (primer[4] && (currentTime-pressTime>7000)) {
      encodeMessage(0,devType[0]);  //Register
      primer[4]=0;
    }
    else if (currentTime<pressTime) {
      pressTime=currentTime; 
    }
    else if (digitalRead(btnPin)) {
      pressed = false;
      primer[0]=0;
      primer[1]=0;
      primer[2]=0;
      primer[3]=0;
      primer[4]=0;
      postRelease=true;
      timeOutCount=0;
      sleepSet(); //Sleep when button is released and measurements complete
    }
  }
  //Check battery if well timed
  else if (batteryFlag && (millis()-pressTime>30000)) {
    batteryFlag=false;
    watchdogCounter=0;
    encodeMessage(2,getVCC());
    sleepSet(); //Sleep when button is released and measurements complete
  }
}

//RF Functions ------------------------------------------------------------------------

void pulse(bool logic) {
  if (logic) { //a one
    digitalWrite(sendPin,HIGH);
    delayMicroseconds(410);  //797us realtime - 720
    digitalWrite(sendPin,LOW);
    delayMicroseconds(10);   //416us realtime - 60
  }
  else { //a zero
    digitalWrite(sendPin,HIGH);
    delayMicroseconds(140);  //416us realtime -320
    digitalWrite(sendPin,LOW);
    delayMicroseconds(30);  //797us realtime - 470
  }
}

void encodeMessage(byte msgType,unsigned long msg) {
  int k;
  //More messages if registering for determining reception
  if (msgType==0) { //If registering, send more repeats for signal quality check
    repQuant=9;
  }
  else {
    repQuant=5;
  }
  msgLength=msgLengths[msgType];
  k=0;
  //construct the message
  for (int i=0; i<8; i++) {  //6 bit preamble with 2 bit msg type - 8 bit total
    bitWrite(msgBuffer[k/8],7-(k%8),bitRead(typePreamble[msgType],7-i));
    k++;
  }
  for (int i=0; i<32; i++) {  //32 bit device ID
    bitWrite(msgBuffer[k/8],7-(k%8),bitRead(devID[0],31-i));
    k++;
  }
  for (int i=0; i<msgLength; i++) {  //72,48,56,72 bit message length
    bitWrite(msgBuffer[k/8],7-(k%8),bitRead(msg,msgLength-1-i));
    k++;
  }
  
  //Send the message
  pulse(1); //For calibration pre-read of reciever
  pulse(0); //For calibration pre-read of reciever
  delay(1);
  for (int rep=0; rep<repQuant; rep++) {
    for (int i=0; i<msgLength+40; i++) {
      pulse(bitRead(msgBuffer[i/8],7-(i%8)));
    }
    pulse(0); //to end the message timing
    delay(1);
  }
  delay(1);
}
