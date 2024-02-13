//========== Tricopter Program ==========

//Libraries
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <QMC5883LCompass.h>
#include "Servo.h"

//********** compass HMC5883L **********
QMC5883LCompass compass;
int x, y, z;
float declinationAngle;
float heading_tilt = 0.0;
float headingDegrees;
float heading, setHeading;
float heading_reference;
float heading_control;
float acc_x, acc_y;
float compensateRoll, compensatePitch;
float cosComRoll, sinComRoll, cosComPitch, sinComPitch;
float Yh, Xh, Ymag_correct, Xmag_correct;
int normXAxis, normYAxis, normZAxis;


//********** mpu 6050 **********
MPU6050 mpu;
MPU6050 accelgyro;
int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float gyroX_filt, gyroY_filt, gyroZ_filt;

float G_Dt = 0.005;

double rad_yaw, rad_pitch, rad_roll;
double roll_kalman, pitch_kalman, yaw_kalman;
double accum_roll = 0;
double accum_pitch = 0;
double k_acc = 0;
double k_gps = 0;
double yaw_deg;
double pitch_deg;
double roll_deg;
double pitch_deg_previous, roll_deg_previous;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_ACCELGYRO

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
boolean interruptLock = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}

//********** brushless motor **********
#define MOTOR1 2
#define MOTOR2 3
#define MOTOR3 4

Servo brushless1;
Servo brushless2;
Servo brushless3;

int motor1;
int motor2;
int motor3;

//********** servo motor **********
Servo myservo;
int servoAngleInit = 106;
int servo;
float yawControlServo;

//********** remote channels **********
volatile int channel1 = 0;
volatile int roll_channel = 0;
volatile int channel2 = 0;
volatile int throttle_channel = 0;
volatile int channel3 = 0;
volatile int pitch_channel = 0;
volatile int channel4 = 0;
volatile int yaw_channel = 0;
volatile int channel5 = 0;
volatile int ch5_channel = 0;

//********** custom variables **********
int ch5, throttle, throttle_input, heading_mode, heading_mode1;
int roll_input, pitch_input, yaw_input;
unsigned long timeProgram, previousTimeProgram;
unsigned long timeServo, previousTimeServo;
char inChar;

void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  Serial3.begin(57600);

  //sensors
  init_MPU();
  compass.init();

  //remote signals
  remote_init();
  delay(50);

  //actuators
  myservo.attach(5);
  servo_setup();
  motor_setup();
}

void loop() 
{
  get_YPR();
  compass_update();
  mapremote();

  if (ch5==1)
  {
    throttle = throttle_channel;
    throttle_input = throttle_channel;
    control_update();
  }

  update_servo();
  update_motor();

  serialEvent();
  timeProgram = micros();
  if (timeProgram - previousTimeProgram >= 100000)
  {
    Serial.print("sudut roll = "); Serial.print(roll_deg); Serial.print("\t");
    Serial.print("sudut pitch = "); Serial.print(pitch_deg); Serial.print("\t");
    Serial.print("sudut yaw = "); Serial.print(headingDegrees); Serial.print("\t");
//    Serial.print("Yh = "); Serial.print(x); Serial.print("\t");
//    Serial.print("Xh = "); Serial.print(y); Serial.print("\t");
    Serial.println();
    previousTimeProgram = micros();
  }
}
