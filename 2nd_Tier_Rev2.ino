#include <RPLidar.h>

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "DFRobot_ICG20660L.h"


// You need to create an driver instance
RPLidar lidar;

#define RPLIDAR_MOTOR 13 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal

SoftwareSerial leo(50, 51);

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define SW_sensorD 8
#define SW_sensorB 9

#define PWM_MTRka 2
#define L_MTRka   4
#define R_MTRka   3

#define PWM_MTRki 5
#define L_MTRki   7
#define R_MTRki   6

#define Btn_Start 33
#define Btn_Reset 35

#define Buzzer 47
#define Lamp 49

#define Prox1 23
#define Prox2 25
#define Prox3 27
#define Prox4 29
#define Prox5 31


//==============define IMU=================
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define M_PI 3.141592653589793238462643
float degree=0;
long int lasttime=0;
long int timepast=0;
float angleX=0;
float angleY=0;
float yawn=0;
float pitch=0;
float roll=0;
long int current =0;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

  sIcg20660SensorData_t  gyro;
  sIcg20660SensorData_t  accel;

DFRobot_ICG20660L_IIC icg(/*addr=*/IIC_ADDR_SDO_H, &Wire);

float DPS = 3.1415926/180.0; // unit: 1dps = 3.1415926/180.0rad/s
float G = 9.80665; //unit: 1G = 9.80665m/s²

//control pins for left and right motors
const int leftSpeed = 9; //means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 5;
const int left1 = 3; //left 1 and left 2 control the direction of rotation of left motor
const int left2 = 2;
const int right1 = 8;
const int right2 = 4;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime_, previousTime_;
int c = 0;

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160; //min PWM value at which motor moves
float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 248; //rough estimate of PWM at the speed pin of the stronger motor, while driving straight 
//and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; //it the car driving forward OR rotate/stationary
bool prevIsDriving = true; //equals isDriving in the previous iteration of void loop()
bool paused = false; //is the program paused

bool objectavoiding = false;
//==============define IMU=================

//==============define song note===========
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
//===============FOR LIDARRR=============
bool kanan_obs = false;
bool kiri_obs = false;
bool depan_obs = false;
bool obs_left = false;
bool obs_right = false;
//================AGV SENSOR=============
int counthex = 0;
int simpanhex[10];
int filterSens=0;

uint32_t inputser;
uint32_t inputser_agv2;

uint32_t pengali3[8] = {0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001};
uint32_t pengali2[8] = {0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000};
uint16_t cek = 0x00;
uint32_t sens = 0x0000;
int count_i = 0;
int hasil = 0;
int hasil2 = 0;
int pembagi = 0;
int pembagi2 = 0;
int nsens = 0;
int hexnsens = 0;
int byteReceived;

long int countunloading = 0;

uint16_t cek_agv2 = 0x00;
uint32_t sens_agv2 = 0x0000;
int count_i_agv2 = 0;
int hasil_agv2 = 0;
int hasil2_agv2 = 0;
int pembagi_agv2 = 0;
int pembagi2_agv2 = 0;
int nsens_agv2 = 0;
int hexnsens_agv2 = 0;
int byteReceived_agv2;
int NAV_COND=0;

byte request1 [] = {
  0x01,
  0x06,
  0x00,
  0x04,
  0x00,
  0x02,
  0x49,
  0xCA

};

byte request [] =
{
  0x01,
  0x03,
  0x00,
  0x00,
  0x00,
  0x02,
  0xC4,
  0x0B
};

//================AGV SENSOR=============

//=================Sensor================
const int pinSensor[12] = {A0, A2, A3, A5, A6, A7, A8, A9, A11, A12, A13, A15};

int SP_SensD[12] = {850, 400, 400, 400, 400, 400, 400, 380, 380, 400, 700, 850};
int SP_SensB[12] = {800, 350, 350, 300, 350, 365, 650, 250, 360, 360, 650, 740};

int     RADC[12]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool    nS[12]    = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int     nADC[12];
String  Ds = "";
int cnt_sens = 0;
float sens_error = 0;

//===============PID=====================
double Sp_sensor, Error, PV, P, Kp, LastPV;
double D1, D2, D3, Kd, Ts = 2, Last_error, D;
double I1, I2, I3, I, Ki, Pd, Pid, NPid;




double Kp_maju, Kd_maju, Ki_maju;
double Kp_mundur, Kd_mundur, Ki_mundur;
int maxspeed_maju, Sppwm_maju, maxspeed_mundur, Sppwm_mundur;
int Sp_pwm, maxspeed, minspeed = 0;
int Pwm_kiri, Pwm_kanan;

//=======================================
int DP[4] = {0, 0, 0, 0};
bool f_maju, f_mundur, f_main = 0;
bool UNLOADING = false;
bool CHANGE_DIRECT = false;
int COUNT_LINE = 0;
int count_detectagv = 0;
int count_stop = 0;
long int countiing=0;
long int detect_obs=0;

//==============Communication====================
String inputString = "", datamasuk = "";
String S_CNC[8] = {"CNC0", "CNC7", "CNC6", "CNC5", "CNC4", "CNC3", "CNC2", "CNC1"};
int a = 0;
int LINE[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//==============Milis============================
//long previousMillis = 0;
long interval = 4000;
unsigned long currentMillis;
int countstart=0;
// declaring variables
const int tonePin = 10;
unsigned long previousMillis = 0;
unsigned long int currentmilisnow=0;
unsigned long int prevdetect=0;
//const long pauseBetweenNotes = 250;   // interval between notes (ms)
//const long noteDuration = 400;        // (ms)
boolean outputTone = false;                // Records current state
const int MelodyLength = 2;
//const int Melody[MelodyLength] = {880, 698};
int MelodyIndex = 0;
//unsigned long currentMillis;

//==============battery==========================
String battery;

//==============song=============================

int thisNote = 0;

int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};

int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

int underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};

int underworld_tempo[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18, 18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};

//===========song======================


unsigned long time_now = 0;
int period = 100;
int waktu = 5000;
void IO_setup() {
  for (int i = 0; i < 12; i++) {
    pinMode(pinSensor[i], INPUT);
  }

  pinMode(Btn_Start, INPUT_PULLUP);
  pinMode(Btn_Reset, INPUT_PULLUP);
  pinMode(SW_sensorD, OUTPUT);
  pinMode(SW_sensorB, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(Lamp, OUTPUT);

  pinMode(L_MTRka, OUTPUT);
  pinMode(R_MTRka, OUTPUT);
  pinMode(L_MTRki, OUTPUT);
  pinMode(R_MTRki, OUTPUT);
  pinMode(PWM_MTRka, OUTPUT);
  pinMode(PWM_MTRki, OUTPUT);

  pinMode(Prox1, INPUT_PULLUP);
  pinMode(Prox2, INPUT_PULLUP);
  pinMode(Prox3, INPUT_PULLUP);
  pinMode(Prox4, INPUT_PULLUP);
  pinMode(Prox5, INPUT_PULLUP);
  Kp_maju = 4.5;
  Kd_maju = 0.01;
  Ki_maju = 0.001;
  Kp_mundur = 4.5;
  Kd_mundur = 0.01;
  Ki_mundur = 0.001;
  maxspeed_maju = 150;
  Sppwm_maju = 130;
  maxspeed_mundur = 150;
  Sppwm_mundur = 130;

}

void setup() {
  //Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(115200);
  leo.begin(9600);

  lidar.begin(Serial3);
  Serial.begin(9600);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  Kp = 4.5;
  Ki = 0.01;
  Kd = 0.001;
  maxspeed = 150;
  Sp_pwm = 130;

  PID_maju(); PID_mundur();
  IO_setup();
  digitalWrite(Buzzer, HIGH);
  delay(100);
  digitalWrite(Buzzer, LOW);
  delay(50);
  digitalWrite(Buzzer, HIGH);
  delay(100);
  digitalWrite(Buzzer, LOW);
  delay(50);
  digitalWrite(Buzzer, HIGH);
  delay(100);
  digitalWrite(Buzzer, LOW);
  delay(2000);

  f_maju = 1;
  f_mundur = 0;
  //f_maju=1;
  COUNT_LINE = 0;
  analogWrite(RPLIDAR_MOTOR, 255);

  f_maju = EEPROM.read(10);
  if(f_maju==1)
  {
  f_maju=1;
  f_mundur=0;
  }
  else
  {
  f_maju=0;
  f_mundur=1;
  }
  
  COUNT_LINE = EEPROM.read(11);
  NAV_COND = EEPROM.read(12);

 // setupvoid();

 // calculateError();
  //       serialEvent3();
}

void loop() {
  currentMillis = millis();
  leo_serial();

  if(f_main==1)
  {
    //lidarr();
    obs_left=false;
    obs_right=false;
  }
  else
  {
    objectavoiding=false;
    obs_right=false;
    obs_left=false;
    depan_obs = false;  
    digitalWrite(Buzzer,LOW);
    analogWrite(RPLIDAR_MOTOR, 0);
  }
  
  
 // IMU_read();


  //led_start();
  //sing(2);
  //sing(1);
  //sing(2);
  //PlayMelody();
  //Serial.print("cek");

  //buzz_on()
  //f_maju=0;
  //f_mundur=1;
  //f_main=1;

  // motor_crontrol(HIGH,HIGH);
  // Wrt_mtr(-100,-80); 1

  //forward();
  currentmilisnow=millis();
    if (f_maju == 1)
    { //if(f_main == 1)
      //avoiding();
      //motor_crontrol(HIGH,LOW);
      if((obs_left==true || obs_right==true) && f_main == 1 )
      {

          digitalWrite(Buzzer,HIGH);
          count_detectagv=0;
          CHANGE_DIRECT=false;
          if(obs_left==true && obs_right==true && f_main==1)
          {
            berhenti();
          }
          else if(obs_left==true && f_main==1)
          {
            float anglenow = angle;
            
            //while((angle-anglenow) == 90)
           // {
            //turn_right();
            /*
            Wrt_mtr(-100,100);
            delay(500);
            forward();
            delay(3000);
            Wrt_mtr(100,-100);
            delay(400);
            forward();
            delay(4000);
            */
          //  Wrt_mtr(100,-100);
          //  delay(500);
          //  forward();
          //  delay(1000);
          //  Wrt_mtr(-100,100);
          //  delay(500);
            
          //  IMU_read();
            obs_left=false;
           // }
          
          }        
          else if(obs_right==true && f_main==1)
          {
           // turn_left();
            obs_right = false;
          }
        
      }
      else
      SerialEven_AGV2();
    }
    if (f_mundur == 1)
    {
      // motor_crontrol(LOW,HIGH);
      SerialEven_AGV1();
    }
  

//Serial.println("trs");
/*
if (f_maju == 1)
    {
      //motor_crontrol(HIGH,LOW);
      SerialEven_AGV2();
    }
    if (f_mundur == 1)
    {
      // motor_crontrol(LOW,HIGH);
      SerialEven_AGV1();
    }
  /*
    if (f_main==1){
    //Serial.print("jalan");

    }
    else{
     berhenti();
    }
  */
  //  int outmotor=80+sens_error;
  //  Serial.println(outmotor);
  //Wrt_mtr((100+Sp_sensor),100);

  //buzz_on();

  /*
    Wrt_mtr(-50,0);
    digitalWrite(SW_sensorD,LOW);
    digitalWrite(SW_sensorB,HIGH);

    // Serial.println(analogRead(A2));
    printADC();
  */

}
