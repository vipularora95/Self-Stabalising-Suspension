#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 
bool blinkState = false;
int mpos[4] = {0, 0, 0, 0};
int panan = 0, rolan = 0;
float buf=.2;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


Servo s0, s1, s2, s3;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  int t = 0;

  s0.attach(4);
  s1.attach(5);
  s2.attach(6);
  s3.attach(7);
  s2.write(180 - mpos[2]);
  s1.write(180 - mpos[1]);
  s0.write(mpos[0]);
  s3.write(mpos[3]);
  delay(500);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial); 
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    delay(5000);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================




// Stabalisation Code

//??///////////////////////////////////////////////////////////////////////////////////////////////
//??///////////////////////////////////////////////////////////////////////////////////////////////
//??///////////////////////////////////////////////////////////////////////////////////////////////
//??///////////////////////////////////////////////////////////////////////////////////////////////




void motor0(int dec)
{
  int up = 180, dwn = 0;
  if (dec == 1 && mpos[0] < 180)
  {
    mpos[0]++;
  }
  else if (dec == 2 && mpos[0] > 0)
  {
    mpos[0]--;
  }
  s0.write(mpos[0]);
}


void motor1(int dec)
{
  int up = 0, dwn = 180;
  if (dec == 1 && mpos[1] < 180)
  {
    mpos[1]++;
  }
  else if (dec == 2 && mpos[1] > 0)
  {
    mpos[1]--;
  }
  s1.write(180 - mpos[1]);
}


void motor2(int dec)
{
  int up = 0, dwn = 180;
  if (dec == 1 && mpos[2] < 180)
  {
    mpos[2]++;
  }
  else if (dec == 2 && mpos[2] > 0)
  {
    mpos[2]--;
  }
  s2.write(180 - mpos[2]);
}

//motor2 up = 0; down = 165;
//motor3 up = 165 ; down = 0;

void motor3(int dec)
{
  int up = 180, dwn = 0;
  if (dec == 1 && mpos[3] < 180)
  {
    mpos[3]++;
  }
  else if (dec == 2 && mpos[3] > 0)
  {
    mpos[3]--;
  }
  s3.write(mpos[3]);
}




void up()
{
  motor0(1);
  motor1(1);
  motor2(1);
  motor3(1);
}


void down()
{
  if(mpos[0] > 0 && mpos[1] > 0 && mpos[2] > 0 && mpos[3] > 0)
  {motor0(2);
  motor1(2);
  motor2(2);
  motor3(2);}
}

void dir1()
{
  if (mpos[0] == 180 || mpos[1] == 180)
  {
    if(rolan>0)
    motor2(2);
    else if(rolan<0)
    motor3(2);
    else
    {
    motor2(2);
    motor3(2);
    }
  }
  else
  {
    if(rolan>0)
    motor1(1);
    else if(rolan<0)
    motor0(1);
    else
    {
      motor0(1);
      motor1(1);
    }
  }
}

void dir5()
{
  if (mpos[2] == 180 || mpos[3] == 180)
  {
    if(rolan>0)
    motor0(2);
    else if(rolan<0)
    motor1(2);
    else
    {
    motor0(2);
    motor1(2);
    }
  }
  else
  {
    if(rolan>0)
    motor3(1);
    else if(rolan<0)
    motor2(1);
    else
    {
      motor3(1);
      motor2(1);
    }
  }
}


void dir7()
{
  if (mpos[1] == 180 || mpos[3] == 180)
  {
    motor2(2);
    motor0(2);
  }
  else
  {
    motor1(1);
    motor3(1);
  }
}



void dir3()
{
  if (mpos[0] == 180 || mpos[2] == 180)
  {
    motor1(2);
    motor3(2);
  }
  else
  {
    motor0(1);
    motor2(1);
  }
}

void dir2()
{
  if (mpos[0] == 180)
  {
    motor1(2);
    motor2(2);
    motor3(2);
  }
  else 
  motor0(1);
}

void dir4()
{
    if (mpos[0] == 180)
  {
    motor1(2);
    motor0(2);
    motor3(2);
  }
  else 
  motor2(1);
}
void dir6()
{
    if (mpos[0] == 180)
  {
    motor1(2);
    motor2(2);
    motor0(2);
  }
  else 
  motor3(1);
}
void dir8()
{
    if (mpos[0] == 180)
  {
    motor0(2);
    motor2(2);
    motor3(2);
  }
  else 
  motor1(1);
}


void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.println(2000 + (ypr[1] * 180 / M_PI));
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);

    
    panan = (ypr[2] * 180 / M_PI);
    rolan = (ypr[1] * 180 / M_PI);
    if (panan < -buf && rolan < -buf)
    {
      dir2();
      //Serial.println(" Dir 4 ");
    }
    else if (panan < -buf && rolan > buf)
    {
      dir8();
      //Serial.println(" Dir 6 ");
    }
    else if (panan > buf && rolan > buf)
    {
      dir6();
      //Serial.println(" Dir 8 ");
    }
    else if (panan > buf && rolan < -buf)
    {
      dir4();
      //Serial.println(" Dir 2 ");
    }
    else
    {
    if (panan < -buf)
    {
      dir1();
      //Serial.println(" Dir 5 ");
    }
    
    if (rolan > buf)
    {
      dir7();
      //Serial.println(" Dir 7 ");
    }
    
    if (panan > buf)
    {
      dir5();
      //Serial.println(" Dir 1 ");
    }
    
    if (rolan < -buf)
    {
      dir3();
      //Serial.println(" Dir 3 ");
    }
    }
//    if (panan>-buf && panan<buf && rolan>-buf && rolan<buf)
//    {
//      down();
//      delay(25);
//    }
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //Serial.print("areal\t");
    //Serial.print(aaReal.x);
    //Serial.print("\t");
    //Serial.print(aaReal.y);
    //Serial.print("\t");
    //Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
