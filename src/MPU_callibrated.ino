#include<Wire.h>

 float RateRoll,RatePitch,RateYaw;
 float RateCallibrationRoll,RateCallibrationPitch,RateCallibrationYaw;
 int   Callibrationumber;
 void gyrosignals(){
//starting transmission
  Wire.beginTransmission(0x68);
//start the low pass filter with 10Hz
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
// set the sensitivity(65.5LSB/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x08);
  Wire.endTransmission();
// storing the measuremnts in memory
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  // request 6 bits from mpu6050
  Wire.requestFrom(0x68,6);
  // read the 16 bit value gyro measurements arround corresponing x,y,z axis
  int16_t gyroX=Wire.read()<<8|Wire.read();
  int16_t gyroY=Wire.read()<<8|Wire.read();
  int16_t gyroZ=Wire.read()<<8|Wire.read();

  RateRoll=(float) gyroX/65.5;
  RatePitch =(float) gyroY/65.5;
  RateYaw =(float) gyroZ/65.5;

 }

  void setup(){
    Serial.begin(57600);
    pinMode(13,OUTPUT);
    digitalWrite(13,HIGH);

    Wire.setClock(400000);//set the clocksped to 400KHz as per the dataspeed
    Wire.begin();
    delay(250);
// start the gyro in power mode
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    // performing callibration measurements
    for(Callibrationumber=0;Callibrationumber<2000;Callibrationumber++){
      gyrosignals();
      RateCallibrationRoll += RateRoll;
      RateCallibrationPitch += RatePitch;
      RateCallibrationYaw += RateYaw;
      delay(1);
    }
    // take the averages and calculate callibration values
    RateCallibrationRoll /=2000;
    RateCallibrationPitch /=2000;
    RateCallibrationYaw /=2000;
  }

  void loop(){
    gyrosignals();
    // corrected values
    RateRoll -= RateCallibrationRoll;
    RatePitch -= RateCallibrationPitch;
    RateYaw -=RateCallibrationYaw;
    Serial.print("rollRate ");
    Serial.print(RateRoll   );
    Serial.print(" \t ");
    Serial.print("pitchRate ");
    Serial.print(RatePitch);
    Serial.print(" \t ");
    Serial.print("YawRate");
    Serial.println(RateYaw);
    delay(50);
  }