#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>
#include <MapleFreeRTOS900.h>
#include <Servo.h>
#include <String>

#define LED_PIN PC13

MPU6050 mpu;
Kalman kalmanX;
Kalman kalmanY;

unsigned long timer;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;
float kalAngleX, kalAngleY;

const float DT = 0.002;
const float AA = 0.98;

String readall()
{
  String s = "";
  char c = 0;
  if (Serial.available() > 0)
  {
    while (Serial.available() > 0)
    {
      c = Serial.read();
      s += c;
      // Serial.print(c);Serial.print("  **");Serial.println((int)c);
    }

    while (c != '\n')
    {
      c = Serial.read();
      if (c != 255)
        s += c;
      // Serial.print(c);Serial.print(" ");Serial.println((int)c);
    }
  }
  return s;
}

float angleY, angleX;

static void vReadMPUTask(void *pvParameters)
{
  Serial.println("Hello!!!");
  for (;;)
  {
    digitalWrite(LED_PIN, HIGH);
    unsigned long now = micros();
    float dt = (now - timer) / 1000000.0;
    timer = now;

    getRollPitch(&mpu, &roll, &pitch);

    kalAngleX = kalmanX.getAngle(roll, gyroX, dt);
    kalAngleY = kalmanY.getAngle(pitch, gyroY, dt);

    // 滤波后的数据
    angleX = AA * kalAngleX + (1 - AA) * roll;
    angleY = AA * kalAngleY + (1 - AA) * pitch;

    Serial.print("A_CYCLEF");
    Serial.print(0);
    Serial.print("F");
    Serial.print(0);
    Serial.print("F");
    Serial.print(0);
    Serial.print("F");
    Serial.print(0);
    Serial.print("F");
    Serial.print(0);
    Serial.print("F");
    Serial.print(0);
    Serial.print("F");
    Serial.print(angleX);
    Serial.print("F");
    // Serial.print(fRollRate); //Serial.print("),\tPitch:");
    Serial.print(angleY);
    Serial.print("F\n");

    vTaskDelay(50);
    digitalWrite(LED_PIN, LOW);
  }
}

Servo aileL, aileR, elev, rudd, thro;
int pitchNeed, rollNeed, throNeed, flap, yaw;
int writeServo(int s)
{
  return 90 - s;
}

static void vServoTask(void *pvParameters)
{
  for (;;)
  {
    aileL.write(writeServo(rollNeed - (int)angleX) + flap);
    aileR.write(writeServo(rollNeed - (int)angleX) - flap);
    elev.write(writeServo((int)angleY - pitchNeed));
    rudd.write(yaw + 90);
    thro.write((int)((float)throNeed / 100 * 260 + 60));
  }
}

static void vReceiveTask(void *pvParameters)
{
  for (;;)
  {
    String str = "";
    if (Serial.available() > 0)
      str = readall() /*,Serial.print("in:"),Serial.println(str)*/;
    int pos;
    String strp, strr, stry, strt, strf;

    pos = str.lastIndexOf('P');
    if (pos != -1)
    {
      strp = str.substring(pos, str.indexOf('\n', pos) - pos + 1);
      pitchNeed = strp.substring(1, str.length() - 1).toFloat();
      //Serial.println(strp);
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
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Welcome!!!");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin();
  mpu.initialize();

  // 设置加速度计量程为 ±16g
  mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_16);

  // 设置陀螺仪量程为 ±2000°/s
  mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_2000);

  // 获取当前欧拉角值作为初始值
  getRollPitch(&mpu, &roll, &pitch);

  // 根据当前欧拉角值计算初始偏移量
  float offsetRoll = 0 - roll;
  float offsetPitch = 0 - pitch;

  kalmanX.setAngle(offsetRoll);
  kalmanY.setAngle(offsetPitch);

  timer = micros();

  thro.attach(PA4);
  aileL.attach(PB1);
  aileL.write(90);
  aileR.attach(PA7);
  aileR.write(90);
  rudd.attach(PB0);
  rudd.write(90);
  elev.attach(PA6);
  elev.write(90);

  xTaskCreate(vReadMPUTask,
              "MPU",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY,
              NULL);
  xTaskCreate(vServoTask,
              "SERVO",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY,
              NULL);
  xTaskCreate(vReceiveTask,
              "RECEIVE",
              configMINIMAL_STACK_SIZE + 100,
              NULL,
              tskIDLE_PRIORITY,
              NULL);
  vTaskStartScheduler();
}

void loop()
{
}

void getRollPitch(MPU6050 *mpu, float *roll, float *pitch)
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accX = (float)ax / 16384.0;
  float accY = (float)ay / 16384.0;
  float accZ = (float)az / 16384.0;

  *roll = atan2(accY, accZ) * 180.0 / PI + 22;
  *pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI - 40;
}
