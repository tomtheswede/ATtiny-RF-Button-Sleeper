/*  
 *   For sending 433MHz RF signals from battery operated ATtiny85
 *   Code by Thomas Friberg (https://github.com/tomtheswede)
 *   Updated 21/04/2017
 */

//Device parameters
const unsigned long devID = 183322503; // 00001010111011011000100111101111 So the message can be picked up by the right receiver
const unsigned long devType = 1; //Reads as "1" corresponding with BTN type

//General variables
const byte sendPin = 2; //RF pin. pin 3 for rfbutton v0.4 which is pb4 so use 4
const byte pwrPin = 1; //Power for external elements pin. pin 6 for rfbutton v0.4 pin 6 is PB1 so use 1
const byte typePreamble[4] = {120,121,122,123}; //01111000 for register, 01111001 for basic message type
long lastTrigger=millis();
bool longPressPrimer=true;
bool longerPressPrimer=true;
bool longestPressPrimer=true;
byte msgType=1;
byte msgLengths[4]={32,8,16,32};
byte msgBuffer[9]; //Max 9 bytes for transmission
int msgLength;
int k;

//Button related
const byte btnPin = 0; //rfButton v0.4 uses pin 5 which is PB0 so use 0 here.
bool lastButtonState=0;
bool buttonState=0;
long buttonTriggerTime=millis();
long currentTime=millis();
bool primer[4]={0,0,0,0};

void setup() { //This code runs as soon as the device is powered on.
  pinMode(sendPin,OUTPUT);
  pinMode(pwrPin,OUTPUT);
  digitalWrite(pwrPin,1);
  pinMode(btnPin,INPUT_PULLUP);
  //PORTB &= ~(1<<PB0);

  encodeMessage(0,devID); //Register on first on
}

void loop() {

  CheckButton();

}

void CheckButton() {
  buttonState=!(digitalRead(btnPin));
  currentTime=millis();
  if (buttonState!=lastButtonState) {
    if (buttonState && currentTime-buttonTriggerTime>300) {
      encodeMessage(1,0);
      buttonTriggerTime=currentTime;
      primer[0]=1;
      primer[1]=1;
      primer[2]=1;
      primer[3]=1;
    }
    else if (!buttonState) {
      primer[0]=0;
      primer[1]=0;
      primer[2]=0;
      primer[3]=0;
    }
  }
  lastButtonState=buttonState;
  if (primer[0] && (currentTime-buttonTriggerTime>600)) {
    encodeMessage(1,1);
    primer[0]=0;
  }
  else if (primer[1] && (currentTime-buttonTriggerTime>1500)) {
    encodeMessage(1,2);
    primer[1]=0;
  }
  else if (primer[2] && (currentTime-buttonTriggerTime>4000)) {
    encodeMessage(1,3);
    primer[2]=0;
  }
  else if (primer[3] && (currentTime-buttonTriggerTime>8000)) {
    encodeMessage(0,devID);  //Register
    primer[3]=0;
  }
}

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
