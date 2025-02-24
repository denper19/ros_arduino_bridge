/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <util/atomic.h>

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA
   
   /* Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER

   /* L298 Motor driver*/
   #define L298_MOTOR_DRIVER
#endif

// #define INTERRUPT_PIN 18  // use pin 2 on Arduino Uno & most boards
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h" 
#include "Wire.h"
MPU6050 mpu;
uint16_t fifoCount;     // count of all bytes currently in FIFO
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;       

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t iqx, iqy, iqz, iqw;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
  case IMU_READ:

    // Serial.print(Serial.available());
    // Serial.print(" ");
    // if(Serial.availableForWrite() > 62)
    // {
      // you can print
      Serial.write((byte*)&iqx, sizeof(iqx));
      Serial.write((byte*)&iqy, sizeof(iqy));
      Serial.write((byte*)&iqz, sizeof(iqz));
      Serial.write((byte*)&iqw, sizeof(iqw));
      Serial.write((byte*)&ax, sizeof(ax));
      Serial.write((byte*)&ay, sizeof(ay));
      Serial.write((byte*)&az, sizeof(az));
      Serial.write((byte*)&gx, sizeof(gx));
      Serial.write((byte*)&gy, sizeof(gy));
      Serial.write((byte*)&gz, sizeof(gz));
      Serial.println("");
    // }

    // Serial.print(iqx);
    // Serial.print(" ");
    // Serial.print(iqy);
    // Serial.print(" ");
    // Serial.print(iqz);
    // Serial.print(" ");
    // Serial.print(iqw);
    // Serial.print(" ");
    // Serial.print(ax);
    // Serial.print(" ");
    // Serial.print(ay);
    // Serial.print(" ");
    // Serial.print(az);
    // Serial.print(" ");
    // Serial.print(gx);
    // Serial.print(" ");
    // Serial.print(gy);
    // Serial.print(" ");
    // Serial.println(gz);
  
    break;

#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));

    /*
    int left_count = readEncoder(LEFT);
    int right_count = readEncoder(RIGHT);

    diffLeft = left_count - prevLeft;
    prevLeft = left_count;

    diffRight = right_count - prevRight;
    prevRight = right_count;

    Serial.write((byte*)&diffLeft, sizeof(diffLeft));
    Serial.write((byte*)&diffRight, sizeof(diffRight));

    Serial.write((byte*)&left_count, sizeof(left_count));
    Serial.write((byte*)&right_count, sizeof(right_count));
    */

    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(10000, true);

  mpu.initialize();
  // pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  mpu.setDMPEnabled(true);

  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  packetSize = mpu.dmpGetFIFOPacketSize();

  // Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;

#ifdef MPU9250
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
#endif

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  #endif
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  GetCurrentFIFOPacket(fifoBuffer, packetSize, 3);
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  iqx = q.x * 100;
  iqy = q.y * 100;
  iqz = q.z * 100;
  iqw = q.w * 100;

  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  // { 
  //   mpu.dmpGetQuaternion(&q, fifoBuffer);

  //   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //   // we want the quaternion values in integer format

  //   iqx = q.x * 100;
  //   iqy = q.y * 100;
  //   iqz = q.z * 100;
  //   iqw = q.w * 100;
  // }
  // iqx = 121;
  // iqy = 121;
  // iqz = 121;
  // iqw = 121;
  // ax = -136;
  // ay = -136;
  // ay = -136;
  // gx = 148;
  // gy = 148;
  // gz = 148;

  // mpuInterrupt = false;
  // mpuIntStatus = mpu.getIntStatus();

  // // Check for DMP data ready interrupt (this should happen frequently)
  // Serial.println("START");
  // if (mpuIntStatus & 0x02) {   

  //   // wait for correct available data length, should be a VERY short wait    
  //   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  //   // read a packet from FIFO
  //   mpu.getFIFOBytes(fifoBuffer, packetSize);
          
  //   // track FIFO count here in case there is > 1 packet available
  //   // (this lets us immediately read more without waiting for an interrupt)
  //   fifoCount -= packetSize;
  //     // get quaternion values in easy matrix form: w x y z
  //   mpu.dmpGetQuaternion(&q, fifoBuffer);

  //   iqx = q.x * 100;
  //   iqy = q.y * 100;
  //   iqz = q.z * 100;
  //   iqw = q.w * 100;
  // }
  // Serial.println("NOT BLOCKED");

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}

uint8_t GetCurrentFIFOPacket(uint8_t* data, uint8_t length, uint8_t max_loops)
{
  fifoCount = mpu.getFIFOCount();
  int GetPacketLoop = 0;
  int OuterPacketLoop = 0;
  if(fifoCount != length && OuterPacketLoop <= 3)
  {
    mpu.resetFIFO();
    delay(1);

    fifoCount = mpu.getFIFOCount();
    GetPacketLoop = 0;

    while (fifoCount < length && GetPacketLoop < max_loops)
    {
      fifoCount = mpu.getFIFOCount();
      GetPacketLoop++;
    }

    if(GetPacketLoop >= max_loops)
    {
      return 0;
    }

    OuterPacketLoop++;
  }

  if(OuterPacketLoop < 3)
  {
    mpu.getFIFOBytes(data, length);
    return 1;
  }

  return 0;
}