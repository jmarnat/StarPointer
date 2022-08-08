// MPU should be connected as so:
// SCL: PIN A5
//

#define TRUE_SAMPLE_RATE_MS 10 // should be 10
#define SMOOTHING_SAMPLES   20 // should be 20

#include <Wire.h>

//int16_t samples[SMOOTHING_SAMPLES][3];

const int MPU_addr = 0x68;

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int64_t AcX_sum, AcY_sum, AcZ_sum;
int16_t AcX_avg, AcY_avg, AcZ_avg;

int minVal = 265;
int maxVal = 402;

double x; double y; double z;
byte res;


unsigned long int t_prev = millis();
unsigned long int t_curr;
int i_sample;






void initSampling() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
}

void takeSample() {
  if (isReadyForSmoothing()) return;

  initSampling();

  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  AcX_sum += AcX;
  AcY_sum += AcY;
  AcZ_sum += AcZ;
  i_sample += 1;
}

bool isReadyForSmoothing() {
  return i_sample >= SMOOTHING_SAMPLES;
}


void setup() {
  delay(3000);
  Serial.begin(9600);
//  Serial.println("Starting sequence.");

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  res = Wire.endTransmission(true);

//  if (res == 0) {
//    Serial.println("MPU connected.");
//  } else {
//    Serial.println("ERROR: MPU not found !");
//  }

  AcX_sum = 0;
  AcY_sum = 0;
  AcZ_sum = 0;
  i_sample = 0;


  Serial.println("X,Y,Z,aX,aY,aZ");
  initSampling();
}

void loop() {
  t_curr = millis();
  if (t_curr > (t_prev + (TRUE_SAMPLE_RATE_MS * i_sample))) {
//    Serial.print("sample no ");
//    Serial.println(i_sample);
    takeSample();
    //t_prev = t_curr;

//    Serial.print(AcX);
//    Serial.print(",");
//    Serial.print(AcY);
//    Serial.print(",");
//    Serial.print(AcZ);
//    Serial.print(",");
//    Serial.print(AcX_avg);
//    Serial.print(",");
//    Serial.print(AcY_avg);
//    Serial.print(",");
//    Serial.print(AcZ_avg);
//    Serial.println();

    if (isReadyForSmoothing()) {
      // Serial.println("smoothing!");

      AcX_avg = AcX_sum / SMOOTHING_SAMPLES;
      AcY_avg = AcY_sum / SMOOTHING_SAMPLES;
      AcZ_avg = AcZ_sum / SMOOTHING_SAMPLES;
      AcX_sum = 0;
      AcY_sum = 0;
      AcZ_sum = 0;

      Serial.print(AcX_avg);
      Serial.print(",");
      Serial.print(AcY_avg);
      Serial.print(",");
      Serial.print(AcZ_avg);
      Serial.println();
      
      i_sample = 0;
      t_prev = t_curr;
    }
  }





  /*
    struct RecGyro rec_gyro;                              // = {AcX, AcY, AcZ};
    memset(&rec_gyro, 0, sizeof(struct RecGyro));
    rec_gyro.acx = AcX;
    rec_gyro.acy = AcY;
    rec_gyro.acz = AcZ;

    if (fifo->isFull(fifo)) {
      Serial.println("INFO: FIFO full");

    } else {
      fifo->add(fifo, &rec_gyro);
    }
  */

  /*
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
    Serial.print("AngleX= "); Serial.println(x);
    Serial.print("AngleY= "); Serial.println(y);
    Serial.print("AngleZ= "); Serial.println(z);
    Serial.println("-----------------------------------------");
  */




    /*
  Serial.print(AcX);
  Serial.print(",");
  Serial.print(AcY);
  Serial.print(",");
  Serial.print(AcZ);
  Serial.println();
    */

  /*
    Serial.print(GyX);
    Serial.print(",");
    Serial.print(GyY);
    Serial.print(",");
    Serial.print(GyZ);
    Serial.println();
  */


}
