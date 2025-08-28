/* TEENSY 4.0 ANGLE MODE DRONE FLIGHT CONTROLLER :
   Two chnages done
 In pidequation PREverror and PrevIterm is changed to  pass by reference
 In kalman_1d function KalmanState and KalmanUncertainty is also changed to pass by Reference

*/


#include<Wire.h>
#include <PulsePosition.h>

 float RateRoll,RatePitch,RateYaw;
 float RateCallibrationRoll,RateCallibrationPitch,RateCallibrationYaw;
 int   Callibrationumber;
 
PulsePositionInput ReceiverInput(RISING); // Detect rising edges of PPM signal
float receiverValue[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 8 channels initialized to 0
int channelnumber = 0;
float Voltage;

uint32_t LoopTimer;

float DesiredrateRoll,DesiredratePitch,DesiredrateYaw;
float ErrorRateroll,ErrorRatepitch,ErrorRateyaw;
float Inputroll,Inputthrottle,Inputpitch,Inputyaw;
float PrevErrorRateroll,PrevErrorRatepitch,PrevErrorRateyaw;
float PrevItermRateroll,PrevItermRatepitch,PrevItermRateyaw;
float PIDReturn[]={0,0,0};
float Prateroll=0.6;float Pratepitch=Prateroll;float Prateyaw=2;
float Irateroll=3.5;float Iratepitch=Irateroll;float Irateyaw=12;
float Drateroll=0.03;float Dratepitch=Drateroll;float Drateyaw=0;
// motor variables
float motorinput1,motorinput2,motorinput3,motorinput4;

float AccX,AccY,AccZ;
float AngleRoll,AnglePitch;

// predicted angles and uncertainty angles
float KalmanAngleRoll=0,KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0,KalmanUncertaintyAnglePitch=2*2;
// angle prediction ,Uncertainty prediction
float Kalman1DOutput[]={0,0};
float DesiredAngleRoll,DesiredAnglePitch;
float ErrorAngleRoll,ErrorAnglePitch;
// for pid
float PrevErrorAngleRoll,PrevErrorAnglePitch;
float PrevItermAngleRoll,PrevItermAnglePitch;
float PAngleRoll=2;float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;
// kalman fuction
void kalman_1d(float &KalmanState,float &KalmanUncertainty,float KalmanInput,
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

// battery voltage monitor function
void battery_voltage(){
  Voltage=(float)analogRead(4)/28.18;//calculate x and put

}
// receiver function
void read_receiver() {
  channelnumber = ReceiverInput.available(); // Get number of available channels
  if (channelnumber > 0) {
    for (int i = 1; i <= channelnumber; i++) {
      receiverValue[i - 1] = ReceiverInput.read(i); // Read pulse width for channel i
    }
  }
}
// gyro function
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
//  PID FUNCTION
 void pidEquation (float Error,float P,float I,float D,float &PrevError,float &PrevIterm){
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if(Iterm>400) Iterm=400;
  else if (Iterm<-400) Iterm =-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput=Pterm+Dterm+Iterm;
  if(PIDOutput>400) PIDOutput=400;
  else if (PIDOutput<-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2 ]=Iterm;
 }
 // PID reset function
 void resetPid(void){
 PrevErrorRateroll=0;PrevErrorRatepitch=0;PrevErrorRateyaw=0;
 PrevItermRateroll=0;PrevItermRatepitch=0;PrevItermRateyaw=0;

 PrevErrorAngleRoll=0;PrevErrorAnglePitch=0;
 PrevItermAngleRoll=0;PrevItermAnglePitch=0;
 }
// setup function
 void setup (){
  // red led
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
//  gyro setup
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
     // Turn on the green led after the finishing off the process
    pinMode(6,OUTPUT);
    digitalWrite(6,HIGH);

    battery_voltage();
  // red led
    if(Voltage>7.5) digitalWrite(5,LOW);
  else digitalWrite(5,HIGH);
  // avoid accidental start
  ReceiverInput.begin(14);
  while(receiverValue[2]<1020 || receiverValue[2]>1050){
  read_receiver();
  delay(4);
  }
 LoopTimer=micros();
 }
/***************************************************************/
// loop function
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

   read_receiver();

   DesiredAngleRoll=0.10*(receiverValue[0]-1500);
   DesiredAnglePitch=0.10*(receiverValue[1]-1500);
   Inputthrottle=receiverValue[2];
   DesiredrateYaw=0.15*(receiverValue[3]-1500);

   ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;  
   ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

   pidEquation (ErrorAngleRoll,PAngleRoll,IAngleRoll,DAngleRoll,
                 PrevErrorAngleRoll,PrevItermAngleRoll) ;
   DesiredrateRoll=PIDReturn[0];
   PrevErrorAngleRoll=PIDReturn[1];
   PrevItermAngleRoll=PIDReturn[2];   
  pidEquation (ErrorAnglePitch,PAnglePitch,IAnglePitch,DAnglePitch,
                 PrevErrorAnglePitch,PrevItermAnglePitch) ;
   DesiredratePitch=PIDReturn[0];
   PrevErrorAnglePitch=PIDReturn[1];
   PrevItermAnglePitch=PIDReturn[2];  

  ErrorRateroll=DesiredrateRoll-RateRoll;
  ErrorRatepitch=DesiredratePitch-RatePitch;
  ErrorRateyaw=DesiredrateYaw-RateYaw;

  pidEquation (ErrorRateroll,Prateroll,Irateroll,Drateroll,PrevErrorRateroll,
                PrevItermRateroll);
  Inputroll=PIDReturn[0];
  PrevErrorRateroll=PIDReturn[1];
  PrevItermRateroll=PIDReturn[2];
  pidEquation (ErrorRatepitch,Pratepitch,Iratepitch,Dratepitch,PrevErrorRatepitch,
                PrevItermRatepitch);
  Inputpitch=PIDReturn[0];
  PrevErrorRatepitch=PIDReturn[1];
  PrevItermRatepitch=PIDReturn[2];
  pidEquation (ErrorRateyaw,Prateyaw,Irateyaw,Drateyaw,PrevErrorRateyaw,
                PrevItermRateyaw);
  Inputyaw=PIDReturn[0];
  PrevErrorRateyaw=PIDReturn[1];
  PrevItermRateyaw=PIDReturn[2];
  // limit he throttle to 80%
  if (Inputthrottle>1800) Inputthrottle=1800;

  motorinput1=1.024*(Inputthrottle-Inputroll-Inputpitch-Inputyaw);
motorinput2=1.024*(Inputthrottle-Inputroll+Inputpitch+Inputyaw);
motorinput3=1.024*(Inputthrottle+Inputroll+Inputpitch-Inputyaw);
motorinput4=1.024*(Inputthrottle+Inputroll-Inputpitch+Inputyaw);
if(motorinput1>2000) motorinput1=1999;
if(motorinput2>2000) motorinput2=1999;
if(motorinput3>2000) motorinput3=1999;
if(motorinput4>2000) motorinput4=1999;
// keep the motors runnning
int ThrottleIdle=1180;
if (motorinput1<ThrottleIdle) motorinput1=ThrottleIdle;
if (motorinput2<ThrottleIdle) motorinput2=ThrottleIdle;
if (motorinput3<ThrottleIdle) motorinput3=ThrottleIdle;
if (motorinput4<ThrottleIdle) motorinput4=ThrottleIdle;
// turn off the motors at throttle lowest position
int ThrottleCutOff=1000;
if(receiverValue[2]<1050){
 motorinput1=ThrottleCutOff;
 motorinput2=ThrottleCutOff;
 motorinput3=ThrottleCutOff;
 motorinput4=ThrottleCutOff;
 resetPid();
}
analogWrite(1,motorinput1);
analogWrite(2,motorinput2);
analogWrite(3,motorinput3);
analogWrite(4,motorinput4);
// monitor again the battery voltage
battery_voltage ();
 if(Voltage>7.5) digitalWrite(5,LOW);
  else digitalWrite(5,HIGH);
  // finish the 250Hz control loop
while (micros()-LoopTimer<4000);
LoopTimer=micros();

}
