// 鸣谢：Devymex
// http://www.gnu.org/licenses/gpl-3.0.en.html
// 相关文档参见作者于知乎专栏发表的原创文章：
// http://zhuanlan.zhihu.com/devymex/20082486

// 连线方法
// MPU-UNO
// VCC-VCC
// GND-GND
// SCL-A5
// SDA-A4
// INT-2 (Optional)
#include <Servo.h>
#include <Kalman.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Math.h>
#include <String>

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// using namespace Servo;
float fRad2Deg = 57.295779513f; // 将弧度转为角度的乘数
const int MPU = 0x68;           // MPU-6050的I2C地址
const int nValCnt = 7;          // 一次读取寄存器的数量

const int nCalibTimes = 1000; // 校准时读数的次数
int calibData[nValCnt];       // 校准数据
unsigned long nLastTime = 0;  // 上一次读数的时间
float fLastRoll = 0.0f;       // 上一次滤波得到的Roll角
float fLastPitch = 0.0f;      // 上一次滤波得到的Pitch角
Kalman kalmanRoll;            // Roll角滤波器
Kalman kalmanPitch;           // Pitch角滤波器

int pitchNeed, rollNeed, throNeed, flap; // 机师所需的俯仰姿态
bool first;
float zitai[5];
String s1, rem;
Servo aileL, aileR, elev, rudd, thro;
float yaw = 0;
int writeServo(int s)
{
  return 90 - s;
}

void setup()
{
  Wire.begin();
  Serial.begin(115200); // 初始化串口，指定波特率
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  // accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  // accelgyro.setClockSource(MPU6050_CLOCK_INTERNAL);
  thro.attach(PB1);
  aileL.attach(PA3);
  aileL.write(90);
  aileR.attach(PA1);
  aileR.write(90);
  rudd.attach(PB0);
  rudd.write(90);
  elev.attach(PA6);
  elev.write(90);

  // flap.attach(PA6);flap.write(0);
  delay(1000);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Calibration();         // 执行校准
  nLastTime = micros();  // 记录当前时间
  pinMode(PC13, OUTPUT); // STM32填PC13，Arduino(AVR等)填13就可以了（下同）
}
bool blinkState;
String readall()
{
  String s = "";
  char c = 0;
  if (Serial.available() > 0)
  {
    //Serial.println("Read");
    while (Serial.available() > 0)
    {
      //Serial.println("Reading");
      c = Serial.read();
      s += c;
    }

    while (c != '\n')
    {
      //Serial.println("Reading111");
      c = Serial.read();
      s += c;
    }
  }
  return s;
}
void loop()
{
  int readouts[nValCnt] = {0};
  // ReadAccGyr(readouts); //读出测量值
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  readouts[0] = ax;
  readouts[1] = ay;
  readouts[2] = az;
  readouts[3] = gx;
  readouts[4] = gy;
  readouts[5] = gz;
  float realVals[7];
  Rectify(readouts, realVals); // 根据校准的偏移量进行纠正

  // 计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); // 计算Roll角
  if (realVals[1] > 0)
  {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); // 计算Pitch角
  if (realVals[0] < 0)
  {
    fPitch = -fPitch;
  }

  // 计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  // 对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  // 跟据滤波值计算角度速
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;

  // 更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  // 更新本次测的时间
  nLastTime = nCurTime;

  // 向串口打印输出Roll角和Pitch角，运行时在Arduino的串口监视器中查看
  Serial.print("A_CYCLEF");
  Serial.print(realVals[0]);Serial.print("F");
  Serial.print(realVals[1]);Serial.print("F");
  Serial.print(realVals[2]);Serial.print("F");
  Serial.print(realVals[3]);Serial.print("F");
  Serial.print(realVals[4]);Serial.print("F");
  Serial.print(realVals[5]);Serial.print("F");
  Serial.print(fNewRoll);Serial.print("F");
  //Serial.print(fRollRate); //Serial.print("),\tPitch:");
  Serial.print(fNewPitch); Serial.print("F\n");
  // Serial.print(fPitchRate);// Serial.print(")\n");
  //-----------------------华丽的分割线——仿Airbus Fly By Wire飞控部分-----------------------
  String str = "";
  if (Serial1.available() > 0)
    str = readall() /*,Serial.print("in:"),Serial.println(str)*/;

  int pos;
  String strp, strr, stry, strt, strf;

  pos = str.lastIndexOf('P');
  if (pos != -1)
  {
    strp = str.substring(pos, str.indexOf('\n', pos) - pos + 1);
    pitchNeed = strp.substring(1, str.length() - 1).toFloat();
  }
  pos = str.lastIndexOf('R');
  if (pos != -1)
  {
    strr = str.substring(pos, str.indexOf('\n', pos) - pos + 1);
    rollNeed = strr.substring(1, str.length() - 1).toFloat();
  }
  pos = str.lastIndexOf('Y');
  if (pos != -1)
  {
    stry = str.substring(pos, str.indexOf('\n', pos) - pos + 1);
    yaw = stry.substring(1, str.length() - 1).toFloat();
  }
  pos = str.lastIndexOf('T');
  if (pos != -1)
  {
    strt = str.substring(pos, str.indexOf('\n', pos) - pos + 1);
    throNeed = strt.substring(1, str.length() - 1).toInt();
  }
  pos = str.lastIndexOf('F');
  if (pos != -1)
  {
    strf = str.substring(pos, str.indexOf('\n', pos) - pos + 1);
    flap = strf.substring(1, str.length() - 1).toInt();
  }

  aileL.write(writeServo(rollNeed - fNewRoll) + flap);
  aileR.write(writeServo(rollNeed - fNewRoll) - flap);
  elev.write(writeServo(fNewPitch - pitchNeed));
  rudd.write(yaw + 90);
  thro.write((float)throNeed / 100 * 260 + 60);
  blinkState = !blinkState;
  digitalWrite(PC13, blinkState);
  delay(20);
}

// 向MPU6050写入一个字节的数据
// 指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal)
{
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

// 对大量读数进行统计，校准平均偏移量
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  // 先求和
  for (int i = 0; i < nCalibTimes; ++i)
  {
    int mpuVals[nValCnt];
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpuVals[0] = ax;
    mpuVals[1] = ay;
    mpuVals[2] = az;
    mpuVals[3] = gx;
    mpuVals[4] = gy;
    mpuVals[5] = gz;
    for (int j = 0; j < nValCnt; ++j)
    {
      valSums[j] += mpuVals[j];
    }
  }
  // 再求平均
  for (int i = 0; i < nValCnt; ++i)
  {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; // 设芯片Z轴竖直向下，设定静态工作点。
}

// 算得Roll角。算法见文档。
float GetRoll(float *pRealVals, float fNorm)
{
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

// 算得Pitch角。算法见文档。
float GetPitch(float *pRealVals, float fNorm)
{
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

// 对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals)
{
  for (int i = 0; i < 3; ++i)
  {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i)
  {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}
