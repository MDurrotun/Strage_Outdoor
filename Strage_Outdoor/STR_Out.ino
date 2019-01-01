///7E 07 81 00 E2 48 49 DD 

//7E    incomingByte[0]   = start delimiter
//00 07 incomingByte[1-2] = length
//81    incomingByte[3]   = frame type
//00 00 incomingByte[4-5] = 16 bit source address
//48    incomingByte[6]   = RSSI
//02    incomingByte[7]   = option
//48 49 incomingByte[8-n] = RF data "HI"
//A3    incomingByte[n+1] = check sum

#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <pt.h>
#include <EEPROM.h>


AltSoftSerial Bee;
SoftwareSerial SerialIMU(6, 7); //RX|TX
SoftwareSerial BLE(10, 11); 

#define putus       0
#define tersambung  1


///////////////////////////////////////////Global Variable//////////////////////////////

///////////////////////////////////////////Indicator//////////////////////////////////
const int led1=2, led2=3, led3=4, led4=5, led6=A0, led7=A1, led8=A2, led5=A3, buzz=A5;
int tick2;

//////////////////////////////////////////IMU////////////////////////////////////////
int YPR[3],yaw,sdt,newSdt,tmpSdt,pos,tick,errorSdt,i,arah,Q[8];
long sdtAwal;
unsigned char Re_buf[8],counter=0;
unsigned char sign=0;

/////////////////////////////////////////XBee////////////////////////////////////////
byte incomingByte[11];
float percentage=999,rssi=999;
unsigned char statuss=0;
int y;
byte message[] = {0x7E, 0x00, 0x07, 0x01, 0x01, 0xFF, 0xFF, 0x00, 0x48, 0x49, 0x6E};

////////////////////////////////////////Bluetooth BLE////////////////////////////////
unsigned char data_received;


//////////////////////////////Protothread/////////////////////////////
static struct pt pt1, pt2, pt3, pt4;

static int protothread1(struct pt *pt, int interval);
static int protothread2(struct pt *pt, int interval);
static int protothread3(struct pt *pt, int interval);
static int protothread4(struct pt *pt, int interval);


//////////////////////////////XBee/////////////////////////////////////
void resetIncomingByte();
void readRSSI();

///////////////////////////////INDICATOR///////////////////////////////
void resetLED();
void buzzLed();
void buzzNoLed();
void buzzOn();

///////////////////////////////IMU//////////////////////////////////////
void readIMU();
void convIMU();
void resetQ();
void flagQ();
void ledQuadrant(int arah);


////////////////////////////Machine Learning//////////////////////////
void booting();
void machineLearning();


void setup()
{
  Serial.begin(38400);
  Bee.begin(9600);
  BLE.begin(57600);
  SerialIMU.begin(115200);
  
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  pinMode(led6, OUTPUT);
  pinMode(led7, OUTPUT);
  pinMode(led8, OUTPUT);
  pinMode(buzz, OUTPUT);

  PT_INIT(&pt1);  // initialise the two
  PT_INIT(&pt2);  // protothread variables
  PT_INIT(&pt3);  // protothread variables
  PT_INIT(&pt4);  // protothread variables
 
  booting();      //booting + kalibrasi awal device

  for(int wait=0; wait<=1000; wait++){
    BLE.listen();
    if(BLE.available()) data_received=BLE.read(); 
  
    if(data_received=='#'||data_received=='@'){
      EEPROM.write(0, data_received); 
    }
    Serial.println("gogogo");
  }
}

void loop()
{
  
  if(EEPROM.read(0)=='@'){       //@ char = 64 desimal --> SAR Device
    Serial.print("SAR Device\t");
    Serial.println(EEPROM.read(0));
    
    BLE.listen();
    if(BLE.available()) data_received=BLE.read(); 

    if(data_received=='#'){
      EEPROM.write(0, data_received); 
    }
    else if(data_received=='*'){ // * = ringing buzzer
      buzzOn();
      data_received=0;
    }

    machineLearning();
        
  }
  else if(EEPROM.read(0)=='#'){  //# char = 35 desimal --> Lost Device
    Serial.print("Lost Device\t");
    Serial.println(EEPROM.read(0));
    
    BLE.listen();
    if(BLE.available()) data_received=BLE.read(); 

    if(data_received=='@'){
      EEPROM.write(0, data_received); 
    }
    else if(data_received=='*'){ // * = ringing buzzer
      buzzOn();
      data_received=0;
    }
    
    protothread1(&pt1, 300);
  }

  //Receiver or SAR device
  //machineLearning();
}

static int protothread1(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) { // never stop 
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    sendData();
  }
  PT_END(pt);
}

static int protothread2(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) { // never stop 
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    readRSSI();    
  }
  PT_END(pt);
}

static int protothread3(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    buzzNoLed(); 
  }
  PT_END(pt);
}

static int protothread4(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    ledPutar(); 
  }
  PT_END(pt);
}

void resetIncomingByte(){
    for(int i=0; i<=10; i++){
        incomingByte[i] = 0;
    }
}

void sendData(){
  Bee.write(message, sizeof(message));
}

void readRSSI(){
  for(int i=0; i<=10; i++){
      if(Bee.available()) {
        incomingByte[i] = Bee.read();
        //Serial.print(incomingByte[i], HEX);
        //Serial.print(" ");
      }
    }

    //Serial.println(" ");
  
    if(incomingByte[0]==0x7E&&incomingByte[8]==0x48){  
       y=0; 
       statuss=tersambung;
       percentage = float ((incomingByte[6]-40)/60.00)*100;
       
       if(percentage<0)percentage=0;
       //Serial.print(" \t RSSI Percentage = ");
       //Serial.print(percentage); 
       resetIncomingByte();
       /*Serial.print("RSSI Dec = ");
       Serial.println(incomingByte[6]);  //decimal
       Serial.print("RSSI Hex= ");
       Serial.println(incomingByte[6], HEX);  //Hex*/
    }  
    else{
      y++;  
      if(y>100){ statuss=putus; y=0;}    
    }

    //Serial.print(" \t statuss= ");
    //Serial.println(statuss); 
}

void ledPutar(){
  tick2++;
  if(tick2<=10)digitalWrite(led4, HIGH);
  else if(tick2>10&&tick2<=20)digitalWrite(led5, HIGH);
  else if(tick2>20&&tick2<=30)digitalWrite(led6, HIGH);
  else if(tick2>30&&tick2<=40)digitalWrite(led7, HIGH);
  else if(tick2>40&&tick2<=50)digitalWrite(led8, HIGH);
  else if(tick2>50&&tick2<=60)digitalWrite(led1, HIGH);
  else if(tick2>60&&tick2<=70)digitalWrite(led2, HIGH);
  else if(tick2>70&&tick2<=80)digitalWrite(led3, HIGH);
  else if(tick2>80&&tick2<=110)resetLED();
  else tick2=0;
}

void buzzLed()
{
    tick++;
    if(percentage>=0 && percentage<=40){
        if(tick<=5){
          digitalWrite(buzz, HIGH);digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH); digitalWrite(led4, HIGH); 
          digitalWrite(led5, HIGH); digitalWrite(led6, HIGH); digitalWrite(led7, HIGH); digitalWrite(led8, HIGH);}
        else if(tick>5 && tick<=10){
          digitalWrite(buzz, LOW);digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
          digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); digitalWrite(led8, LOW);}
        else tick=0;
     }
     else if(percentage>40 && percentage<=60){
        if(tick<=10){
          digitalWrite(buzz, HIGH);digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH); digitalWrite(led4, HIGH); 
          digitalWrite(led5, HIGH); digitalWrite(led6, HIGH); digitalWrite(led7, HIGH); digitalWrite(led8, HIGH);}
        else if(tick>10 && tick<=15){
          digitalWrite(buzz, LOW);digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
          digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); digitalWrite(led8, LOW);}
        else tick=0;
     }
     else if(percentage>60 && percentage<=80){
        if(tick<=15){
          digitalWrite(buzz, HIGH);digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH); digitalWrite(led4, HIGH); 
          digitalWrite(led5, HIGH); digitalWrite(led6, HIGH); digitalWrite(led7, HIGH); digitalWrite(led8, HIGH);}
        else if(tick>15 && tick<=20){
          digitalWrite(buzz, LOW);digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
          digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); digitalWrite(led8, LOW);}
        else tick=0;
     }
      else if(percentage>80 && percentage<=100){
        if(tick<=20){
          digitalWrite(buzz, HIGH);digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH); digitalWrite(led4, HIGH); 
          digitalWrite(led5, HIGH); digitalWrite(led6, HIGH); digitalWrite(led7, HIGH); digitalWrite(led8, HIGH);}
        else if(tick>20 && tick<=25){
          digitalWrite(buzz, LOW);digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
          digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); digitalWrite(led8, LOW);}
        else tick=0;
     }
}

void buzzNoLed()
{
    if(statuss==tersambung){
        tick++;
        if(percentage>=0 && percentage<=40){
            if(tick<=5){digitalWrite(buzz, HIGH);}
            else if(tick>5 && tick<=10){digitalWrite(buzz, LOW);}
            else tick=0;
         }
         else if(percentage>40 && percentage<=60){
            if(tick<=10){digitalWrite(buzz, HIGH);}
            else if(tick>10 && tick<=15){digitalWrite(buzz, LOW);}
            else tick=0;
         }
         else if(percentage>60 && percentage<=80){
            if(tick<=15){digitalWrite(buzz, HIGH);}
            else if(tick>15 && tick<=20){digitalWrite(buzz, LOW);}
            else tick=0;
         }
          else if(percentage>80 && percentage<=100){
            if(tick<=20){digitalWrite(buzz, HIGH);}
            else if(tick>20 && tick<=25){digitalWrite(buzz, LOW);}
            else tick=0;
         }
    }
}

void buzzOn(){
  digitalWrite(buzz, HIGH);
  digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH); digitalWrite(led4, HIGH); 
  digitalWrite(led5, HIGH); digitalWrite(led6, HIGH); digitalWrite(led7, HIGH); digitalWrite(led8, HIGH);  
  delay(700);
  digitalWrite(buzz, LOW);
  digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
  digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); digitalWrite(led8, LOW);
  delay(300);
  digitalWrite(buzz, HIGH);
  digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH); digitalWrite(led4, HIGH); 
  digitalWrite(led5, HIGH); digitalWrite(led6, HIGH); digitalWrite(led7, HIGH); digitalWrite(led8, HIGH);  
  delay(700);
  digitalWrite(buzz, LOW);
  digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
  digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); digitalWrite(led8, LOW);
}

void readIMU(){
   while (SerialIMU.available()) {   
    Re_buf[counter]=(unsigned char)SerialIMU.read();
    if(counter==0&&Re_buf[0]!=0xAA) return;            
    counter++;       
    if(counter==8)               
    {    
       counter=0;                 
       sign=1;
    }      
  }
  
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0xAA && Re_buf[7]==0x55)       
     {           
            YPR[0]=(Re_buf[1]<<8|Re_buf[2])/100;   
//            YPR[1]=(Re_buf[3]<<8|Re_buf[4])/100;
//            YPR[2]=(Re_buf[5]<<8|Re_buf[6])/100;
            yaw = YPR[0];        
   }
  } 
}

void resetLED(){
  digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
  digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); analogWrite(led8, 0);  
}

void convIMU(){
  readIMU();
  if(yaw>=0) sdt = yaw; 
  else sdt = 360 + yaw;

  tmpSdt = sdt - sdtAwal;
  
  if(tmpSdt<0) newSdt = 360 + tmpSdt;
  else newSdt = tmpSdt;

  /*Serial.print("yaw:");
  Serial.print(yaw); 
  Serial.print("\t");
  Serial.print("sdt:");
  Serial.print(sdt); 
  Serial.print("\t"); 
  Serial.print("newSdt:"); 
  Serial.print(newSdt); 
  Serial.print("\t");*/ 
  Serial.print("newSdt = ");
  Serial.println(newSdt);
}

void resetQ(){
  for(int i=0; i<8; i++)Q[i]=0;
}

void flagQ(){
  if(newSdt>=339 || newSdt<=23)Q[0]=1;
  else if(newSdt>=24 && newSdt<=68)Q[1]=1;
  else if(newSdt>=69 && newSdt<=113)Q[2]=1;
  else if(newSdt>=114 && newSdt<=158)Q[3]=1;
  else if(newSdt>=159 && newSdt<=203)Q[4]=1;
  else if(newSdt>=204 && newSdt<=248)Q[5]=1;
  else if(newSdt>=249 && newSdt<=293)Q[6]=1;
  else if(newSdt>=294 && newSdt<=338)Q[7]=1;
}

void ledQuadrant(int arah){
  if(arah>=339 || arah<=23){digitalWrite(led4, HIGH);digitalWrite(led5, HIGH);}
  else if(arah>=24 && arah<=68){digitalWrite(led5, HIGH);digitalWrite(led6, HIGH);}
  else if(arah>=69 && arah<=113){digitalWrite(led6, HIGH);digitalWrite(led7, HIGH);}
  else if(arah>=114 && arah<=158){digitalWrite(led7, HIGH);digitalWrite(led8, HIGH);}
  else if(arah>=159 && arah<=203){digitalWrite(led8, HIGH);digitalWrite(led1, HIGH);}
  else if(arah>=204 && arah<=248){digitalWrite(led1, HIGH);digitalWrite(led2, HIGH);}
  else if(arah>=249 && arah<=293){digitalWrite(led2, HIGH);digitalWrite(led3, HIGH);}
  else if(arah>=294 && arah<=338){digitalWrite(led3, HIGH);digitalWrite(led4, HIGH);}
}

void booting(){ 
  SerialIMU.listen();
  for(int i=0; i<6; i++){
    readIMU();
    digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
    digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); digitalWrite(led8, LOW);
    delay(500);
    digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(led3, HIGH); digitalWrite(led4, HIGH); 
    digitalWrite(led5, HIGH); digitalWrite(led6, HIGH); digitalWrite(led7, HIGH); digitalWrite(led8, HIGH);
    delay(500);
  }
  
  for(int i=0; i<200; i++){
    readIMU();
    sdtAwal += yaw;
    Serial.print(yaw);
    Serial.print("\t"); 
    Serial.print("sdtAwal++ = ");
    Serial.println(sdtAwal);
  }
  sdtAwal = sdtAwal/200;

  if(sdtAwal<0) sdtAwal = 360 + sdtAwal; 
  
  digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(led3, LOW); digitalWrite(led4, LOW); 
  digitalWrite(led5, LOW); digitalWrite(led6, LOW); digitalWrite(led7, LOW); analogWrite(led8, 0);
}

void machineLearning(){
  protothread2(&pt2, 100); //rssi xbee
  //protothread3(&pt3, 50); //buzzer no blink led
  //protothread4(&pt4, 10);  //led Putar
  
  resetQ();
  while((Q[0]!=1||Q[1]!=1||Q[2]!=1||Q[3]!=1||Q[4]!=1||Q[5]!=1||Q[6]!=1||Q[7]!=1) && (statuss==tersambung)){
    SerialIMU.listen();
    protothread2(&pt2, 100); //rssi xbee
    protothread3(&pt3, 50);  //buzzer no blink led 
    protothread4(&pt4, 10);  //led Putar
    
    convIMU(); 
    //Serial.print(newSdt); Serial.print(" "); Serial.print(percentage); Serial.println(" ");
     
    flagQ();
    
    if(percentage<rssi){
      rssi = percentage;
      pos  = newSdt;     
    }    
  }
  
  while(statuss==tersambung){
    SerialIMU.listen();
    protothread3(&pt3, 50); //buzzer no blink led 
    protothread2(&pt2, 100); //rssi xbee
    convIMU();
    //Serial.print(newSdt); Serial.print(" "); Serial.print(pos); Serial.print(" ");  Serial.print(errorSdt);  Serial.print(" ");  Serial.print(arah);  Serial.println(" "); 
    errorSdt = newSdt - pos;
  
    if(errorSdt<0) arah = 360 + errorSdt;
    else arah = errorSdt;

    ledQuadrant(arah);
    if(millis()%50==0)resetLED();
  }
  resetLED();
  digitalWrite(buzz, LOW);
}
