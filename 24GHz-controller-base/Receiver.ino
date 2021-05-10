// Receive
#include <RS485.h>
#include <SoftwareSerial.h>
#include <INA219.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

INA219 ina219_A;
LiquidCrystal_I2C lcd(0x27,20,4);
char message[32+3+1];   //[maxMsgLen+3+1];

int   ledpin=13;      //blinkt wenn UB da ist
bool  state13=0;     //status dieser LED13

float ub = 0;       //ina219_A
char  sub[5];
float ib = 0;       //ina219_A
char  sib[5];

int   curs = 0;

void setup()
{
  lcd.init();
  Serial.begin(9600);
  Serial.println("System Startup - Receiver");

  RS485_Begin(28800);
  ina219_A.begin();
  pinMode(ledpin, OUTPUT);
  state13=!state13;  
  digitalWrite(ledpin,state13);

  //Serial.println("LCD-Out");
  lcd.backlight();
  //Nachricht ausgeben
  lcd.setCursor(0,0);
  lcd.print(" joy-IT");
  lcd.setCursor(0,1);
  lcd.print(" I2C Serial LCD");
  
}

void loop()
{
  

  state13=!state13;  
  digitalWrite(ledpin,state13);
  //if(RS485_ReadPlainMessage(fAvailable,fRead,message))
  if(RS485_ReadMessage(fAvailable,fRead, message))
  {
    curs = 0;
    Serial.print("Receiving:");
    Serial.println(message);
    if (message[0]=='X') {
      curs = 1;
    }
      
    lcd.setCursor(0,curs);
    lcd.print(message);
    
  }
  //if(RS485_ReadPlainMessage(fAvailable,fRead,message))
  //if(RS485_ReadMessage(fAvailable,fRead, message))
  //{
  //  curs = 1;
  //  Serial.print("Receiving:");
  //  Serial.println(message);
  //  lcd.setCursor(0,curs);
  //  lcd.print(message);
  //
  
  }
  state13=!state13;  
  digitalWrite(ledpin,state13);

}
