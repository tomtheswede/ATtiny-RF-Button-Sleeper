/*  
 *   For sending 433MHz RF signals from battery operated ATtiny85
 *   Code by Thomas Friberg (https://github.com/tomtheswede)
 *   Updated 21/04/2017
 */

//Device parameters
const unsigned long devID = 183322503; // 00001010111011011000100111101111 So the message can be picked up by the right receiver
const unsigned long devType = 1; //Reads as "1" corresponding with BTN type

//General variables
const byte pwrPin = 1; //Power for external elements pin. pin 6 for rfbutton v0.4 pin 6 is PB1 so use 1

//RF related
const byte sendPin = 2; //RF pin. pin 3 for rfbutton v0.4 which is pb4 so use 4
const byte typePreamble[4] = {120,121,122,123}; //01111000 for register, 01111001 for basic message type
byte msgLengths[4]={32,8,16,32};
byte msgBuffer[9]; //Max 9 bytes for transmission
int msgLength;

//Button related
long currentTime=millis();

//Interrupt variables
volatile const byte btnPin = 0; //rfButton v0.4 uses pin 5 which is PB0 so use 0 here.
volatile bool primer[5]={0,0,0,0,0};
volatile bool pressed=0;
volatile bool buttonState=0;
volatile unsigned long pressTime=0;
volatile const unsigned int reTriggerDelay=50; //minimum time in millis between button presses to remove bad contact
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) //OR
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) //AND

void setup() { //This code runs as soon as the device is powered on.
  pinMode(sendPin,OUTPUT);
  pinMode(pwrPin,OUTPUT);
  digitalWrite(pwrPin,1);
  pinMode(btnPin,INPUT_PULLUP);

  encodeMessage(0,devID); //Register on first on

  digitalWrite(pwrPin,0); //Power down devices between button pushes
  //Set pin to start interrupts
  sbi(GIMSK,PCIE); //Turn on interrupt
  sbi(PCMSK,PCINT0); //set pin affected by interupt - PCINT0 corresponds to PB0 or pin 5
}

void loop() {

  CheckButton();

}

//Button functions --------------------------------------------

ISR(PCINT0_vect) {
  // This is called when the interrupt occurs
  buttonState=!(digitalRead(btnPin));
  if (buttonState && (millis()-pressTime>reTriggerDelay)) { //if pressed in
    pressed=true;
    primer[0]=1;
    primer[1]=1;
    primer[2]=1;
    primer[3]=1;
    primer[4]=1;
    pressTime=millis();
    //encodeMessage(1,5);
  }
}

void CheckButton() {
  if (pressed && digitalRead(btnPin)) {
    pressed = false;
    digitalWrite(pwrPin,0); //Turn off ancillaries
    primer[0]=0;
    primer[1]=0;
    primer[2]=0;
    primer[3]=0;
    primer[4]=0;
  }
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
    else if (primer[3] && (currentTime-pressTime>4000)) {
      encodeMessage(1,4);
      primer[3]=0;
    }
    else if (primer[4] && (currentTime-pressTime>8000)) {
      encodeMessage(0,devID);  //Register
      primer[4]=0;
    }
  }
}

//RF Functions ------------------------------------------------------------------------

void pulse(bool logic) {
  if (logic) {
    digitalWrite(sendPin,HIGH);
    delayMicroseconds(720);  //797us realtime
    digitalWrite(sendPin,LOW);
    delayMicroseconds(60);   //416us realtime
  }
  else {
    digitalWrite(sendPin,HIGH);
    delayMicroseconds(320);  //416us realtime
    digitalWrite(sendPin,LOW);
    delayMicroseconds(470);  //797us realtime
  }
}

void encodeMessage(byte msgType,unsigned long msg) {
  int k;
  msgLength=msgLengths[msgType];
  k=0;
  //construct the message
  for (int i=0; i<8; i++) {  //6 bit preamble with 2 bit msg type - 8 bit total
    bitWrite(msgBuffer[k/8],7-(k%8),bitRead(typePreamble[msgType],7-i));
    k++;
  }
  for (int i=0; i<32; i++) {  //32 bit device ID
    bitWrite(msgBuffer[k/8],7-(k%8),bitRead(devID,31-i));
    k++;
  }
  for (int i=0; i<msgLength; i++) {  //72,48,56,72 bit message length
    bitWrite(msgBuffer[k/8],7-(k%8),bitRead(msg,msgLength-1-i));
    k++;
  }
  
  //Send the message
  pulse(1); //For calibration pre-read of reciever
  pulse(0); //For calibration pre-read of reciever
  delay(2);
  for (int rep=0; rep<5; rep++) {
    for (int i=0; i<msgLength+40; i++) {
      pulse(bitRead(msgBuffer[i/8],7-(i%8)));
    }
    pulse(0); //to end the message timing
    delay(2);
  }
  delay(2);
}
