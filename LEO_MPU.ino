#include <SoftwareSerial.h>
#include <Wire.h>

#define ROTATION_LEFT 4 //направление вращения левого колеса
#define VOLTAGE_LEFT 5 //управляющее напряжение левого колеса
#define VOLTAGE_RIGHT 6 //управляющее напряжение правого колеса
#define ROTATION_RIGHT 7 //направление вращения правого колеса

#define debug 0

#define temp 0x41
#define Gyro_X 0x43
#define Gyro_Y 0x45
#define Gyro_Z 0x47
#define Acc_X 0x3B
#define Acc_Y 0x3D
#define Acc_Z 0x3F

const int mpu_addr=0x68;  // I2C address of the MPU-6050
//Some of those boards have a pull-down resistor at AD0 (address = 0x68), others have a pull-up resistor (address = 0x69).
int16_t AcX,AcY,AcZ;

SoftwareSerial mySerial(9, 8);

float speed = 0; //требуемая скорость (см/с)

///////////////////////////////////////////////////////////////////////////////
///////////   Счетчик прерываний энкодера /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
unsigned long int left_cnt, left_cnt0 = 0; 
unsigned long int right_cnt, right_cnt0 = 0; 
unsigned long int time_left0 = 0; 
unsigned long int time_right0 = 0;

//подсчет отверстий на энкодере левого колеса
void left_wheel(void)
{
  unsigned long int time_left = micros();
  unsigned long int dt_left = time_left - time_left0;
  time_left0 = time_left;

  if (dt_left > 100)
    left_cnt++;
}

//подсчет отверстий на энкодере правого колеса
void right_wheel(void)
{
  unsigned long int time_right = micros();
  unsigned long int dt_right = time_right - time_right0;
  time_right0 = time_right;

  if (dt_right > 100)
    right_cnt++;
}
///////////////////////////////////////////////////////////////////////////////

double Read_reg(uint16_t addr)
{
  Wire.beginTransmission(mpu_addr);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr,2);
  double X=Wire.read()<<8|Wire.read();
  Wire.endTransmission(true);
  return X;
}

void Write_reg(uint16_t addr, uint16_t data)
{
  Wire.beginTransmission(mpu_addr);
  Wire.write(addr);
  Wire.write(data>>8);
  Wire.write(data&0xFF);
  Wire.endTransmission(true);
}

volatile float ax;
volatile float ay;
volatile float az;
float b = 0.1;


float w_x = 0;
float w_y = 0;
float w_z = 0;

float angle_x = 0;
float angle_y = 0;
float angle_z = 0;

float angle_x0 = 0;
float angle_y0 = 0;
float angle_z0 = 0;
float b0 = 0.0001;

//////////////////////////////////////////////////////////////////////////////////////////////
unsigned long int old_time = 0;

void setup() 
{
  pinMode(9, INPUT);
  pinMode(8, OUTPUT);
  mySerial.begin(9600); // BT serial

  Serial.begin(9600);
  Serial.println("???");

  Wire.begin();
  Wire.setClock(100000UL); // Set I2C frequency to 400kHz
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B);
  Wire.write(0); // wake up the mpu6050
  Wire.endTransmission(true);

  pinMode(ROTATION_LEFT, OUTPUT);
  pinMode(VOLTAGE_LEFT, OUTPUT);
  pinMode(VOLTAGE_RIGHT, OUTPUT);
  pinMode(ROTATION_RIGHT, OUTPUT);
  
  digitalWrite(ROTATION_LEFT, HIGH);
  digitalWrite(VOLTAGE_LEFT, LOW);
  digitalWrite(VOLTAGE_RIGHT, LOW);
  digitalWrite(ROTATION_RIGHT, HIGH);

  //attachInterrupt(digitalPinToInterrupt(3),left_wheel,CHANGE); //обработчик прерываний левого колеса
  //attachInterrupt(digitalPinToInterrupt(2),right_wheel,CHANGE); //обработчик прерываний правого колеса
 
  old_time = micros();
  Serial.println("OK");
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Управление с помощью кнопок по BT

float voltage_left = 0; 
float voltage_right = 0; //управляющее напряжение

void remote_control(void)
{
  if (mySerial.available() > 0)
  {
    char cmd = mySerial.read();
    Serial.print((char)cmd); // debug

    if (cmd == '0')
    {
      Serial.println(" Start");
      speed = 0.06;
    }
    else if (cmd == 'X')
    {
      Serial.println(" Stop");
      voltage_left = 0;
      voltage_right = 0;
      speed = 0;
    }
    else if (cmd == 'F')
    {
      Serial.println(" Вперёд");
      digitalWrite(ROTATION_LEFT, HIGH);
      digitalWrite(ROTATION_RIGHT, HIGH);
    }  
    else if (cmd == 'B')
    {
      Serial.println(" Назад");
      digitalWrite(ROTATION_LEFT, LOW);
      digitalWrite(ROTATION_RIGHT, LOW);
    }  
    else if (cmd == 'L')
    {
      Serial.println(" Налево");
      left_cnt += 10; //60
    }
    else if (cmd == 'R')
    {
      Serial.println(" Направо");
      right_cnt += 10; //30
    }
    else if (cmd == 'Q')
    {
      Serial.println(" Быстрее");
      speed *= 1.05; //1.1
      Serial.print("Speed = "); 
      Serial.println(speed*100);
    }
    else if (cmd == 'S')
    {
      Serial.println(" Медленнее");
      speed /= 1.05; //1.1
      Serial.print("Speed = "); 
      Serial.println(speed*100);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

long int error_S = 0;
float error_V_left = 0;
float error_V_right = 0;
float error_V_left_new = 0;
float error_V_right_new = 0;
float d_error_V_left = 0;
float d_error_V_right = 0;
float a_left; //ускорение левого
float a_right; //ускорение правого
unsigned long int dtime;
unsigned long int time;
float V_left = 0;
float V_right = 0;
float S_left = 0; 
float S_right = 0;

void error_calc(void)
{
  float dcnt_left = left_cnt - left_cnt0;
  left_cnt0 = left_cnt;

  float dcnt_right = right_cnt - right_cnt0;
  right_cnt0 = right_cnt;

  S_left = 0.7*S_left + 0.3*(6*3.14*(dcnt_left/40)); 
  S_right = 0.7*S_right + 0.3*(6*3.14*(dcnt_right/40));

  V_left = 0.7*V_left + 0.3*(S_left/dtime*1000); //...*0.99
  V_right = 0.7*V_right + 0.3*(S_right/dtime*1000);

  error_S = left_cnt - right_cnt;

  error_V_left = 0.7*error_V_left + 0.3*(V_left - speed);
  d_error_V_left = error_V_left - error_V_left_new;
  error_V_left_new = error_V_left;

  error_V_right = 0.9*error_V_right + 0.1*(V_right - speed);
  d_error_V_right = error_V_right - error_V_right_new;
  error_V_right_new = error_V_right;

  a_left = d_error_V_left/dtime;
  /*if (A > A_max) 
    A_max = A;
  if (A < A_min) 
    A_min = A;*/

  a_right = d_error_V_right/dtime;
  /*if (B > B_max) 
    B_max = B;
  if (B < B_min) 
    B_min = B; */ 
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

float KP = 120; //коэффициент пропорционального регулятора (700)
float KI = 0.05; //коэффициент интегрального регулятора
float KD = 0; //коэффициент дифференциального регулятора
int N = 0; //счетчик прохождения цикла
void control(void)
{
  if (speed<0.01){
    voltage_left = 0;
    voltage_right = 0;
    //voltage_left -= (error_S*KI - (a_left*KD));
    //voltage_right -= (-error_S*KI - (a_right*KD));
  }
  else
  {
    voltage_left -= (error_V_left*KP + error_S*KI - (a_left*KD));
    voltage_right -= (error_V_right*KP - error_S*KI - (a_right*KD));
  }
  N += 1;

  if (voltage_left > 255)  
    voltage_left = 255;
  if (voltage_right > 255) 
    voltage_right = 255;
  if (voltage_left < 0)    
    voltage_left = 0;
  if (voltage_right < 0)   
    voltage_right = 0;

  analogWrite(VOLTAGE_RIGHT, round(voltage_right));
  analogWrite(VOLTAGE_LEFT, round(voltage_left));

  if(N > 10){
    //Serial.print ("voltage_left = ");
    //Serial.print (voltage_left);
    Serial.print (", V_left = ");
    Serial.print (V_left*100);
    //Serial.print (", error_V_left = ");
    //Serial.print (error_V_left*100);
    //Serial.print (", voltage_right = ");
    //Serial.print (voltage_right);
    Serial.print (", V_right = ");
    Serial.println (V_right*100);
    //Serial.print (", error_V_right = ");
    //Serial.print (error_V_right*100);

    //Serial.print (", S_left = ");
    //Serial.print (S_left);
    //Serial.print (", S_right = ");
    //Serial.print (S_right);
    //Serial.print (", error_S = ");
    //Serial.println (error_S);
    N = 0;
  }
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void loop() 
{
//  Serial.println("?");
  time = micros();
  dtime = time - old_time;

  if(dtime > 10000) // timer
  {
    old_time = time;

    remote_control();
    error_calc();
    control();
  }

  w_x = Read_reg(Gyro_X)/250.0;
  w_y = Read_reg(Gyro_Y)/250.0;
  w_z = Read_reg(Gyro_Z)/250.0;

  angle_x += w_x - angle_x0;
  angle_y += w_y - angle_y0;
  angle_z += w_z - angle_z0;

  angle_x0 = (1-b)*angle_x0 + b*w_x;
  angle_y0 = (1-b)*angle_y0 + b*w_y;
  angle_z0 = (1-b)*angle_z0 + b*w_z;

  Serial.print(w_x);
  Serial.print(";  ");
  Serial.print(w_y);
  Serial.print(";  ");
  Serial.println(w_z);
  delay(100);

}
