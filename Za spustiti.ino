#include <SPI.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Gyroscope.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.9;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 20;
//Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0.9;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02 ;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 10;               //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_aX = 0.9;               //Gain setting for the aX P-controller (1.3)
float pid_i_gain_aX = 0.04;              //Gain setting for the aX I-controller (0.3)
float pid_d_gain_aX = 20;
//Gain setting for the aX D-controller (15)
int pid_max_aX = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_aY = pid_p_gain_aX;  //Gain setting for the aY P-controller.
float pid_i_gain_aY = pid_i_gain_aX;  //Gain setting for the aY I-controller.
float pid_d_gain_aY = pid_d_gain_aX;  //Gain setting for the aY D-controller.
int pid_max_aY = pid_max_aX;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_aZ = pid_p_gain_aX;                //Gain setting for the aZ P-controller. //4.0
float pid_i_gain_aZ = pid_i_gain_aX ;               //Gain setting for the aZ I-controller. //0.02
float pid_d_gain_aZ = pid_d_gain_aX;               //Gain setting for the aZ D-controller.
int pid_max_aZ = pid_max_aX;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Gyroscope gyro(42);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
RF24 radio(7, 8);
int broj;
const uint64_t rxAddr[6] = {1};
#define MAX_Signal 2000
#define MIN_Signal 700
#define MOTOR_PINur 9
#define MOTOR_PINul 10 // dod
#define MOTOR_PINdr 11 // dod 
#define MOTOR_PINdl 12 // dod
Servo motor_ur;
Servo motor_ul;
Servo motor_dr;
Servo motor_dl;
int servoSignal = MIN_Signal;
int servoSignalMore = MIN_Signal; // dod
int servoSignalLess = MIN_Signal; // dod
int inkrement = 75;
float yaw, pitch, roll, aX, aY, aZ;
int batVoltage;
int batPin = 8;
int loop_counter;
float battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

byte highByte, lowByte;
float esc_1, esc_2, esc_3, esc_4;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_i_mem_aX, pid_aX_setpoint, gyro_aX_input, pid_output_aX, pid_last_aX_d_error;
float pid_i_mem_aY, pid_aY_setpoint, gyro_aY_input, pid_output_aY, pid_last_aY_d_error;
float pid_i_mem_aZ, pid_aZ_setpoint, gyro_aZ_input, pid_output_aZ, pid_last_aZ_d_error;

int start, timepassed, flag;
int turnIncrement = 25;
int UpIncrement = 50;
int DownIncrement = 50;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //Serial.begin(4800);
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.

  battery_voltage = (analogRead(0) + 65) * 1.2317;

  // Gyro
  attachInterrupt(2, dmpDataReady, RISING);
  gyro.bootUp();
  // Radio and Motors
  radio.begin();
  radio.openReadingPipe(0, rxAddr[0]);
  radio.startListening();
  motor_ur.attach(MOTOR_PINur);
  motor_ul.attach(MOTOR_PINul);
  motor_dr.attach(MOTOR_PINdr);
  motor_dl.attach(MOTOR_PINdl);
  motor_ur.writeMicroseconds(MIN_Signal);
  motor_ul.writeMicroseconds(MIN_Signal);
  motor_dr.writeMicroseconds(MIN_Signal);
  motor_dl.writeMicroseconds(MIN_Signal);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(120);
  start = millis();
  flag = 0;
  pinMode(42, OUTPUT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (analogRead(batPin) < 190) digitalWrite(42, HIGH);
  timepassed = millis() - start;
  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.

  if (mpuInterrupt) {
    GyroRead(&gyro, mpuInterrupt, yaw, pitch, roll, aX, aY, aZ);
    //  Serial.print("YPR: ");
    //  Serial.print(yaw);
    //  Serial.print(" ");
    //  Serial.print(pitch);
    // Serial.print(" ");
    // Serial.println(roll);
    gyro_roll_input = (gyro_roll_input * 0.8) + ((roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.8) + ((pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.8) + ((yaw / 57.14286) * 0.2);
    gyro_aX_input = aX;
    gyro_aY_input = aY;
    gyro_aZ_input = aZ;
  }

  //Reset the pid controllers for a bumpless start.
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;

  if (timepassed > 3000 && flag == 0) {
    pid_roll_setpoint = 0 ;
    pid_pitch_setpoint = 0;
    pid_yaw_setpoint = 0;
    pid_aX_setpoint
    pid_aY_setpoint
    pid_aZ_setpoint
    flag = 1;
  }
  else if (flag == 1) {
    //PID inputs are known. So we can calculate the pid output.
    calculate_pid();

    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //0.09853 = 0.08 * 1.2317.
    // battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    //Turn on the led if battery voltage is to low.
    //if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(12, HIGH);

    // nas daljinac
    if (radio.available())  {
      radio.read(&broj, sizeof(broj));
      switch (broj) {
        case 1:
          // desno
          motor_ur.writeMicroseconds(esc_1 - turnIncrement);
          motor_ul.writeMicroseconds(esc_2 + turnIncrement);
          motor_dr.writeMicroseconds(esc_3 - turnIncrement);
          motor_dl.writeMicroseconds(esc_4 + turnIncrement);
          // Serial.println(broj);
          break;
        case 2:
          // nazad
          motor_ur.writeMicroseconds(esc_1 + turnIncrement);
          motor_ul.writeMicroseconds(esc_2 + turnIncrement);
          motor_dr.writeMicroseconds(esc_3 - turnIncrement);
          motor_dl.writeMicroseconds(esc_4 - turnIncrement);
          //Serial.println(broj);
          break;
        case 3:
          //napred
          motor_ur.writeMicroseconds(esc_1 - turnIncrement);
          motor_ul.writeMicroseconds(esc_2 - turnIncrement);
          motor_dr.writeMicroseconds(esc_3 + turnIncrement);
          motor_dl.writeMicroseconds(esc_4 + turnIncrement);
          //Serial.println(broj);
          break;
        case 4:
          //levo
          motor_ur.writeMicroseconds(esc_1 + turnIncrement);
          motor_ul.writeMicroseconds(esc_2 - turnIncrement);
          motor_dr.writeMicroseconds(esc_3 + turnIncrement);
          motor_dl.writeMicroseconds(esc_4 - turnIncrement);
          //Serial.println(broj);
          break;
        case 5:
          //Up
          // Serial.println(broj);
          servoSignal += UpIncrement;
          if (servoSignal > 2000) servoSignal = 2000;
          break;
        case 6:
          //Down
          // Serial.println(broj);
          servoSignal -= DownIncrement;
          if (servoSignal < 700) servoSignal = 700;
          break;
      }
    }
    // Idle State - Keep balance
    esc_1 = servoSignal - pid_output_pitch + pid_output_roll - pid_output_yaw-pid_output_aZ+pid_output_aY+pid_output_aX; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = servoSignal - pid_output_pitch - pid_output_roll + pid_output_yaw-pid_output_aZ+pid_output_aY-pid_output_aX; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = servoSignal + pid_output_pitch - pid_output_roll - pid_output_yaw-pid_output_aZ-pid_output_aY+pid_output_aX; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = servoSignal + pid_output_pitch + pid_output_roll + pid_output_yaw-pid_output_aZ-pid_output_aY-pid_output_aX; //Calculate the pulse for esc 4 (front-left - CW)
    if (esc_1 < 700) esc_1 = 700;                                         //Keep the motors running.
    if (esc_2 < 700) esc_2 = 700;                                         //Keep the motors running.
    if (esc_3 < 700) esc_3 = 700;                                         //Keep the motors running.
    if (esc_4 < 700) esc_4 = 700;                                         //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;
    //  Serial.print("PID: ");
    //  Serial.print(esc_1);
    //  Serial.print(" ");
    //   Serial.print(esc_2);
    //  Serial.print(" ");
    //   Serial.print(esc_3);
    //  Serial.print(" ");
    //   Serial.println(esc_4);
    motor_ur.writeMicroseconds(esc_1);
    motor_ul.writeMicroseconds(esc_2);
    motor_dr.writeMicroseconds(esc_3);
    motor_dl.writeMicroseconds(esc_4);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid() {
  int offset=0.5;
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  if(pid_error_temp<offset && pid_error_temp>(-offset)) pid_error_temp=0;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  if(pid_error_temp<offset && pid_error_temp>(-offset)) pid_error_temp=0;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  if(pid_error_temp<offset && pid_error_temp>(-offset)) pid_error_temp=0;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;

  //aX calculations
  pid_error_temp = gyro_aX_input - pid_aX_setpoint;
  if(pid_error_temp<offset && pid_error_temp>(-offset)) pid_error_temp=0;
  pid_i_mem_aX += pid_i_gain_aX * pid_error_temp;
  if (pid_i_mem_aX > pid_max_aX)pid_i_mem_aX = pid_max_aX;
  else if (pid_i_mem_aX < pid_max_aX * -1)pid_i_mem_aX = pid_max_aX * -1;

  pid_output_aX = pid_p_gain_aX * pid_error_temp + pid_i_mem_aX + pid_d_gain_aX * (pid_error_temp - pid_last_aX_d_error);
  if (pid_output_aX > pid_max_aX)pid_output_aX = pid_max_aX;
  else if (pid_output_aX < pid_max_aX * -1)pid_output_aX = pid_max_aX * -1;

  pid_last_aX_d_error = pid_error_temp;

  //aY calculations
  pid_error_temp = gyro_aY_input - pid_aY_setpoint;
  if(pid_error_temp<offset && pid_error_temp>(-offset)) pid_error_temp=0;
  pid_i_mem_aY += pid_i_gain_aY * pid_error_temp;
  if (pid_i_mem_aY > pid_max_aY)pid_i_mem_aY = pid_max_aY;
  else if (pid_i_mem_aY < pid_max_aY * -1)pid_i_mem_aY = pid_max_aY * -1;

  pid_output_aY = pid_p_gain_aY * pid_error_temp + pid_i_mem_aY + pid_d_gain_aY * (pid_error_temp - pid_last_aY_d_error);
  if (pid_output_aY > pid_max_aY)pid_output_aY = pid_max_aY;
  else if (pid_output_aY < pid_max_aY * -1)pid_output_aY = pid_max_aY * -1;

  pid_last_aY_d_error = pid_error_temp;

  //aZ calculations
  pid_error_temp = gyro_aZ_input - pid_aZ_setpoint;
  if(pid_error_temp<offset && pid_error_temp>(-offset)) pid_error_temp=0;
  pid_i_mem_aZ += pid_i_gain_aZ * pid_error_temp;
  if (pid_i_mem_aZ > pid_max_aZ)pid_i_mem_aZ = pid_max_aZ;
  else if (pid_i_mem_aZ < pid_max_aZ * -1)pid_i_mem_aZ = pid_max_aZ * -1;

  pid_output_aZ = pid_p_gain_aZ * pid_error_temp + pid_i_mem_aZ + pid_d_gain_aZ * (pid_error_temp - pid_last_aZ_d_error);
  if (pid_output_aZ > pid_max_aZ)pid_output_aZ = pid_max_aZ;
  else if (pid_output_aZ < pid_max_aZ * -1)pid_output_aZ = pid_max_aZ * -1;

  pid_last_aZ_d_error = pid_error_temp;
}


