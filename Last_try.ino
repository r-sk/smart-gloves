#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>


//MPU9250 variables:
#define cali    3
#define store   5
#define nth     6
#define see     7

long accelX, accelY, accelZ;
float Ax, Ay, Az;
float p=0,r=0;

float CL[99];
float max_CL = 0;
uint8_t matched = 0;

//Analog input variables:


float ix = 0, th = 0;
uint16_t min_ix=350, max_ix=700;
uint16_t min_th=350, max_th=700;

//Software Serial variables:
SoftwareSerial bt(10, 9); // RX, TX
int flag = 0;
int c = 0;

//EEPROM variables:
uint8_t index = 0;
uint8_t t1 = 0, t2 = 0;
uint8_t e_data[10];
float e_read[5];

//Feedback variables:
#define Buzz_Pin 11
#define green 4
#define red 13
#define yellow 12

void setup()
{
  pin_modes();
  Serial.begin(115200);
  bt.begin(9600);
  Wire.begin();
  setupMPU();
  calibrate();
}

void loop()
{
  update_data();
  programbody();
  
}

void pin_modes()
{
  pinMode(13, OUTPUT);

  pinMode(cali, INPUT);   digitalWrite(cali, HIGH);
  pinMode(store, INPUT);   digitalWrite(store, HIGH);
  pinMode(see, INPUT);     digitalWrite(see, HIGH);

  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(yellow, OUTPUT);
}

void setupMPU()
{
  Wire.beginTransmission(0x68);   //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B);               //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000);         //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission(); 
   
  Wire.beginTransmission(0x68);   //I2C address of the MPU
  Wire.write(0x1B);               //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000);         //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);   //I2C address of the MPU
  Wire.write(0x1C);               //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000);         //Setting the accel to +/- 2g
  Wire.endTransmission();
  
}

void calibrate()
{
  Serial.println("Calibrating the flex sensors...");
  digitalWrite(yellow,HIGH);
  while(digitalRead(cali)==HIGH);
  if(digitalRead(cali)==LOW)
  {
    digitalWrite(yellow,LOW);
    buzz(2);
    min_ix = analogRead(A1);
    min_th = analogRead(A0);
    delay(1000);
  }

  digitalWrite(yellow,HIGH);
  while(digitalRead(cali)==HIGH);
  if(digitalRead(cali)==LOW)
  {
    digitalWrite(yellow,LOW);
    buzz(2);
    max_th = analogRead(A0);
    delay(1000);
  }

  digitalWrite(yellow,HIGH);
  while(digitalRead(cali)==HIGH);
  if(digitalRead(cali)==LOW)
  {
    digitalWrite(yellow,LOW);
    buzz(2);
    max_ix = analogRead(A1);
    delay(1000);
  }

  buzz(3);
  
  
}

void update_data()
{
  //Update Flex data:
  th = analogRead(A0);
  ix = analogRead(A1);

  th=map(th,min_th,max_th,0,1000);
  ix=map(ix,min_ix,max_ix,0,1000);

  if(th<=800){ th=1; }
  else       { th=0; }

  if(ix<=700){ ix=1; }
  else       { ix=0; }

 
  
  

  //Update Acc data:
  Wire.beginTransmission(0x68);            //I2C address of the MPU
  Wire.write(0x3B);                        //Starting register for Accel Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0x68,6);               //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();    //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read();    //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read();    //Store last two bytes into accelZ
  
  //Process acclerometer data:
  Ax = accelX / 16384.0;
  Ay = accelY / 16384.0; 
  Az = accelZ / 16384.0;

  Ax *= 9.8;
  Ay *= 9.8;
  Az *= 9.8;

  p = atan2 (Ay ,( sqrt ((Ax * Ax) + (Az * Az))));
  r = atan2(-Ax ,( sqrt((Ay * Ay) + (Az * Az))));

  p = p*57.3;
  r = r*57.3;

  //Showing updated data:
  Serial.print( "|y|p|r|ix|th| =\t" + String(p) + "\t" + String(r) + "\t" + String(th) + "\t" + String(ix)+ "\t\t" );
  
}

void programbody()
{
  
  
  


  if (digitalRead(store) == LOW)
  {
    data_storer();
  }

  

  data_comparer();
  data_matcher();

  
}

void encode(float x, uint8_t in)
{
  in *= 2;
  bool sign = 0;
  int temp = 0;
  if (x < 0) {
    sign = 1;
  }
  x = fabs(x);
  temp = int(x);
  e_data[in + 1] = temp;
  e_data[in] = (temp - e_data[in + 1]) / 256;
  if (sign == 1) {
    e_data[in] += 128;
  }

}

void decod(uint8_t idx)
{
  uint8_t temp = 0;
  bool sign = 0;
  idx *= 10;

  for (int i = 0; i < 5; i++)
  {
    temp = 0; sign = 0;
    temp = EEPROM.read(idx + 2 * i);
    if (temp >= 128) {
      sign = 1;
      temp -= 128;
    }
    e_read[i] = temp * 256;
    temp = EEPROM.read(idx + 2 * i + 1);
    e_read[i] += temp;
    if (sign == 1) {
      e_read[i] = -e_read[i];
    }
  }

}

void data_storer()
{
  encode(0, 0); encode(p, 1); encode(r, 2); encode(ix, 3); encode(th, 4);

  Serial.println("\n\nEnter the INDEX to store data in 2 successive enters ;) :p --> ");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait till data recieved
  t1 = Serial.read();
  t1 -= 48;
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait till data recieved
  t2 = Serial.read();
  t2 -= 48;

  index = 10 * t1 + t2;
  index *= 10;

  Serial.print("Index is : ");
  Serial.println(index);

  for (int i = 0; i < 10; i++)
  {
    Serial.print("Writing in : index ");
    Serial.println(index + i);

    EEPROM.write(index + i, e_data[i] );
  }
}

void data_comparer()
{
  matched = 0;
  max_CL = 0;

  for (int i = 0; i < 100; i++)
  {
    decod(i);

    if (p > (e_read[1] - 15) && p < (e_read[1] + 15) && r > (e_read[2] - 15) && r < (e_read[2] + 15) && (ix == e_read[3]) && (th == e_read[4]) )
    {
      CL[i] = 100 - ( fabs(e_read[1] - p) + fabs(e_read[2] - r) + fabs(e_read[3] - ix) + fabs(e_read[4] - th) )*0.8 ;

      if (fabs(CL[i]) > max_CL)
      {
        //Serial.println("max found");
        max_CL = fabs(CL[i]);
        matched = i;
      }

    }
  }
}

void buzz( uint8_t n)
{
  for (uint8_t i = 0; i < n; i++)
  {
    tone(Buzz_Pin, 1000);
    delay(100);
    noTone(Buzz_Pin);
    delay(100);
  }
}

void data_matcher()
{
  switch(matched)
  {
    case 0:
      digitalWrite(green, LOW);  digitalWrite(red, HIGH);   Serial.println("No match found!! ");
      flag = 0;
      break;

    case 1:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("Hi");
      if (c != 1 && flag == 0)
      {buzz(1);flag = 1;                                       c = 1;  bt.write(" Hi\n\r");
      }
      break;

    case 2:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("My");
      if (c != 2 && flag == 0)
      {buzz(1);flag = 1;                                       c = 2;  bt.write(" My\n\r");
      }
      break;

    case 3:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("name is");
      if (c != 3 && flag == 0)
      {buzz(1);flag = 1;                                       c = 3;  bt.write(" name is\n\r");
      }
    break;

    case 4:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("Rashik");
      if (c != 4 && flag == 0)
      {buzz(1);flag = 1;                                       c = 4;  bt.write(" Nishesh\n\r");
      }
    break;

    case 5:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("from");
      if (c != 5 && flag == 0)
      {buzz(1);flag = 1;                                       c = 5;  bt.write(" from\n\r");
      }  
    break;
      
     case 6:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("team oasis");
      if (c != 6 && flag == 0)
      {buzz(1);flag = 1;                                       c = 6;  bt.write(" Team\n\r"); delay(400); bt.write(" Oasis\n\r");
      }
    break;

    case 21:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("the oasis");
      if (c != 7 && flag == 0)
      {buzz(1);flag = 1;                                       c = 7;  bt.write(" the oasis\n\r");
      }
      break;
    
    case 8:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("we");
      if (c != 8 && flag == 0)
      {buzz(1);flag = 1;                                       c = 8;  bt.write(" Wee are\n\r");
      }
    break;

    case 9:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("are");
      if (c != 9 && flag == 0)
      {buzz(1);flag = 1;                                       c = 9;  bt.write(" are\n\r");
      }
    break;

    
     case 10:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("Pulchowk Campus");
      if (c != 10 && flag == 0)
      {buzz(1);flag = 1;                                       c = 10;  bt.write(" Pool\n\r"); delay(500); bt.write(" Choke\n\r"); delay(500); bt.write(" Campus\n\r");
      }
    break;

    
     case 11:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("yes");
      if (c != 11 && flag == 0)
      {buzz(1);flag = 1;                                       c = 11;  bt.write(" yes\n\r");
      }
    break;

    
     case 12:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("no");
      if (c != 12 && flag == 0)
      {buzz(1);flag = 1;                                       c = 12;  bt.write(" no\n\r");
      }
    break;

    
     case 13:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("ok");
      if (c != 13 && flag == 0)
      {buzz(1);flag = 1;                                       c = 13;  bt.write(" ok\n\r");
      }
    break;

    
     case 14:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("not ok");
      if (c != 14 && flag == 0)
      {buzz(1);flag = 1;                                       c = 14;  bt.write(" not ok\n\r");
      }
    break;

    
     case 15:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("love");
      if (c != 15 && flag == 0)
      {buzz(1);flag = 1;                                       c = 15;  bt.write(" love\n\r");
      }
    break;

    
     case 16:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("you");
      if (c != 16 && flag == 0)
      {buzz(1);flag = 1;                                       c = 16;  bt.write(" you\n\r");
      }
    break;

    
     case 17:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("thank you");
      if (c != 17 && flag == 0)
      {buzz(1);flag = 1;                                       c = 17;  bt.write(" Thank U\n\r");  //delay(400); bt.write(" You\n\r");
      }
    break;

    
     case 18:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("namaste");
      if (c != 18 && flag == 0)
      {buzz(1);flag = 1;                                       c = 18;  bt.write(" namaste\n\r");
      }
    break;

    
     case 19:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("bye");
      if (c != 19 && flag == 0)
      {buzz(1);flag = 1;                                       c = 19;  bt.write(" bye\n\r");
      }
    break;

    
     case 20:
      digitalWrite(red, LOW); digitalWrite(green, HIGH);       Serial.println("what");
      if (c != 20 && flag == 0)
      {buzz(1);flag = 1;                                       c = 20;  bt.write(" what\n\r");
      }
    break;

    default:
      Serial.println("nothing matched!!!");
      break;
  }
      
}

//  if (digitalRead(see) == LOW)
//  {
//    Serial.println("\n\n Enter the INDEX:  ");
//    while ( Serial.available() && Serial.read() );  // empty buffer
//    while (!Serial.available());                    // wait till data recieved
//    index = Serial.read();
//    index -= 48;
//    index *= 10;
//
//    Serial.println(EEPROM.read(index));
//    Serial.println(EEPROM.read(index + 1));
//
//    while (Serial.available() && Serial.read());  // empty buffer
//    while (!Serial.available());                  // wait till data recieved
//    while (Serial.available() && Serial.read());  // empty buffer again
//
//  }

