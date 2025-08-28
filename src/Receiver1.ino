#include <PulsePosition.h>

PulsePositionInput ReceiverInput(RISING); // Detect rising edges of PPM signal
float receiverValue[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 8 channels initialized to 0
int channelnumber = 0;

void read_receiver() {
  channelnumber = ReceiverInput.available(); // Get number of available channels
  if (channelnumber > 0) {
    for (int i = 1; i <= channelnumber; i++) {
      receiverValue[i - 1] = ReceiverInput.read(i); // Read pulse width for channel i
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);       // Onboard LED for debugging
  digitalWrite(13, HIGH);    
  ReceiverInput.begin(14);   // Start PPM input on pin 14
}

void loop() {
  read_receiver();
  Serial.print("Number of Channels: ");
  Serial.print(channelnumber);
  Serial.print(" | Roll[us]: ");
  Serial.print(receiverValue[0]);
  Serial.print(" | Pitch[us]: ");
  Serial.print(receiverValue[1]);
  Serial.print(" | Throttle[us]: ");
  Serial.print(receiverValue[2]);
  Serial.print(" | Yaw[us]: ");
  Serial.print(receiverValue[3]);
  Serial.println(); 
  delay(50);       
}