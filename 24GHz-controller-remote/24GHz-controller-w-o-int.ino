// 24GHZ-Controller-w-o-int
// Send
//Funktion:
// setup: 
// 1. timer für 1ms interrupt
// no: 2. timer for 1s interrupt for writing messages to serial monitor
// 3. input TX (mit Zenerdiodepegelwandlung 3,3V)
// 4. input LOCK (mit Zenerdiodenpegelwandlung 3,3V)
// 5. Output für POWER/HEARTBEAT LED
// 6. output für TX LED (HIGH)
// 7. output für LOCK LED
// 8. Output für TX-relais (LOW)
// 9. RS485 (als Sender)
// 10. U/I Sensor 1
// 11. U/I Sensor 2

// Loop
// 1. LED Betriebspannung ansteuern und 0.1/0.9s aus/ein als visueller Watchdog
// 2. TX Überwachung wenn TX, dann TX-LED ein und HIGH to LOW für TX Relay
// 3. 10MHz Lock-Überwachung, wenn Lock, dann Lock LED ein
// 4. sende den Zustand alle eine (1) Sekunde im Empfangsfall, sonst all 1/4 Sekunde
#include <RS485.h>
#include <SoftwareSerial.h>
#include <INA219.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

INA219 ina219_A;
INA219 ina219_B(0x41);


char  Message[32+1] ; //maxMsgLen+1
char  Message2[maxMsgLen+1];

int   ledpin=13;      //blinkt wenn UB da ist
bool  state13=0;     //status dieser LED13
int   ledpin2=12;      //blinkt wenn UB da ist
bool  state12=0;     //status dieser LED12
int   ledpin3=11;      //blinkt wenn UB da ist
bool  state11=0;     //status dieser LED11
int   inputpintx=10;
bool  state10=0;
int   inputpinlck=8;
bool  state08=0;

float ub = 0;       //ina219_A
char  sub[5];
float ib = 0;       //ina219_A
char  sib[5];
float um = 0;       //ina219_B
char  sum[5];
char  rxtx='R';
char  lck='U';

float fX = 0;       //Heading
float fY = 0;       //Rollwinkel
float fZ = 0;       //Nickwinkel
char sX[6];         
char sY[6];
char sZ[6];         

int iSys=0;
int iGyro=0; 
int iAccel=0;
int iMag=0;

void setup()
{
  Serial.begin(9600);
  Serial.println("System Startup - Sender");

  RS485_Begin(28800);
  ina219_A.begin();
  ina219_B.begin();

  pinMode(ledpin, OUTPUT);
  pinMode(ledpin2, OUTPUT);
  pinMode(ledpin3, OUTPUT);
  pinMode(inputpintx, INPUT);
  pinMode(inputpinlck, INPUT);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  for (int i = 0; i <= 10; i++) {
    state13=!state13;  
    digitalWrite(ledpin,state13);
    state12=!state12;  
    digitalWrite(ledpin2,state12);
    state11=!state11;  
    digitalWrite(ledpin3,state11);
    delay(200);    
  }

  //delay(1000);
    
  bno.setExtCrystalUse(true);

}

void loop()
{
  delay(1000);  
  //read ub
  ub = ina219_A.busVoltage();
  ib = ina219_A.shuntCurrent();
  //ib = abs(ib);
  um = ina219_B.busVoltage();
  dtostrf(ub,4,1,sub);
  dtostrf(ib,3,1,sib);
  dtostrf(um,3,1,sum);    
  //Serial.print("ub:");
  Serial.print(ub,1);
  //Serial.print(sub);
  Serial.print(" ib:");
  Serial.print(ib,1);
  Serial.print("um:");
  Serial.print(um,1);
 

  
  state13=!state13;  
  digitalWrite(ledpin,state13);
  //state12=!state12;  
  //digitalWrite(ledpin2,state12);
  //state11=!state11;  
  //digitalWrite(ledpin3,state11);
  
  state10 = digitalRead(inputpintx);
  //Serial.println(state10);
  digitalWrite(ledpin2,state10);
  if (state10){
    rxtx = 'T';
    }
   else {
    rxtx = 'R';
    }
    
  state08 = digitalRead(inputpinlck);
  //Serial.println(state08);
  digitalWrite(ledpin3,state08);
  if (state08){
    lck = 'L';
    }
   else {
    lck = 'U';
    }

  
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  //Serial.print("X: ");
  fX = event.orientation.x;
  //Serial.print(fX, 4);
  //Serial.print(event.orientation.x, 4);
  //Serial.print("\tY: ");
  fY = event.orientation.y;
  //Serial.print(fY, 4);
  //Serial.print(event.orientation.y, 4);
  //Serial.print("\tZ: ");
  fZ = event.orientation.z;
  //Serial.print(fZ, 4);
  //Serial.print(event.orientation.z, 4);
  //Serial.println("");
  dtostrf(fX,5,1,sX);
  dtostrf(fY,3,1,sY);
  dtostrf(abs(fZ),3,1,sZ);
  //             "Msg12345678901234567"
  //               "1234567890123456"
  //strcpy(Message,"12.3V 0.3A RX0.1");
  //sprintf(Message,"%sV %sA %c%c%sX%s Y%s Z%s",sub,sib,rxtx,lck,sum,sX,sY,sZ);
  sprintf(Message,"%sV %sA %c%c%s",sub,sib,rxtx,lck,sum);
  sprintf(Message2,"X%sZ%s %d%d%d%d",sX,sZ,iSys,iGyro,iAccel,iMag);
  
  if(RS485_SendMessage(Message,fWrite,ENABLE_PIN))
  {
    Serial.print("Sending:");
    Serial.print(Message);
  }  
  delay(100);
  if(RS485_SendMessage(Message2,fWrite,ENABLE_PIN))
  {
    Serial.print(" Sending2:");
    Serial.print(Message2);
  }  
  displayCalStatus();
}

/**************************************************************************/
/*
Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
  iSys = system;
  iGyro = gyro;
  iAccel = accel;
  iMag = mag;
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}
