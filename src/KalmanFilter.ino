#include<Wire.h>

float RateRoll,RatePitch,RateYaw;
float RateCallibrationRoll,RateCallibrationPitch,RateCallibrationYaw;
 int   Callibrationumber;
float AccX,AccY,AccZ;
float AngleRoll,AnglePitch;

uint32_t LoopTimer;
// predicted angles and uncertainty angles
float KalmanAngleRoll=0,KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0,KalmanUncertaintyAnglePitch=2*2;
// angle prediction ,Uncertainty prediction
float Kalman1DOutput[]={0,0};

// kalman fuction
void kalman_1d(float KalmanState,float KalmanUncertainty,float KalmanInput,
               float KalmanMeasurement){
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty+=0.004*0.004*4*4;
  
   float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3*3);
  KalmanState=KalmanState+KalmanGain*(KalmanMeasurement-KalmanState);
   KalmanUncertainty=(1-KalmanGain)* KalmanUncertainty;

  //  output KalmanMeasurement=acceleratometer angle // KalmanInput=rtation rate
   Kalman1DOutput[0]=KalmanState;
   Kalman1DOutput[1]=KalmanUncertainty;
   }

  //  read from gyro acceleration and rotation
   void gyrosignals(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  // configure for acceleratometer mode
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
   Wire.beginTransmission(0x68);
  Wire.write(0x3B);
   Wire.endTransmission();
   Wire.requestFrom(0x68,6);

int16_t AccXLSB=Wire.read()<<8|Wire.read();
int16_t AccYLSB=Wire.read()<<8|Wire.read();
int16_t AccZLSB=Wire.read()<<8|Wire.read();

Wire.beginTransmission(0x68);
Wire.write(0x1B);
Wire.write(0x8);
Wire.endTransmission();
Wire.beginTransmission(0x68);
Wire.write(0x43);
Wire.endTransmission();

Wire.requestFrom(0x68,6);
int16_t GyroX=Wire.read()<<8|Wire.read();
int16_t GyroY=Wire.read()<<8|Wire.read();
int16_t GyroZ=Wire.read()<<8|Wire.read();
RateRoll=(float)GyroX/65.5;
RatePitch=(float)GyroY/65.5;
RateYaw=(float)GyroZ/65.5;
// convert the measurements to physical values
// ****** Do callibration here******
AccX=(float)AccXLSB/4096-0.01;
AccY=(float)AccYLSB/4096+0.01;
AccZ=(float)AccZLSB/4096+0.02;

AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

// setup function
void setup (){
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
    LoopTimer=micros();

}
void loop (){
  gyrosignals();
  RateRoll-=RateCallibrationRoll;
  RatePitch-=RateCallibrationPitch;
  RateYaw-=RateCallibrationYaw;
// call the Kalman functions for the output
  kalman_1d(KalmanAngleRoll,KalmanUncertaintyAngleRoll,
            RateRoll,AngleRoll );
   KalmanAngleRoll=Kalman1DOutput[0];
   KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
   
    kalman_1d(KalmanAnglePitch,KalmanUncertaintyAnglePitch,
            RatePitch,AnglePitch );
   KalmanAnglePitch=Kalman1DOutput[0];
   KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
   // check the roll and pitch angles
   Serial.print("Roll Angle []=");
  Serial.print(KalmanAngleRoll);
  Serial.print("Pitch Angle []=");
  Serial.println(KalmanAnglePitch);
  while (micros()-LoopTimer<4000);
  LoopTimer=micros();


}