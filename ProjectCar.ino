#include <ECE3.h>

uint16_t sensorValues[8]; 

const int DARKEST_READING = 1000;

const int left_nslp_pin=31; 
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; 
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

void setup() {
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();

  Serial.begin(9600); 
  delay(2000); 
  
}

void blinkLED() {
  digitalWrite(LED_RF, HIGH);
  delay(250);
  digitalWrite(LED_RF, LOW);
  delay(250);
}

bool hasDetectedCrossbar() {
  for (unsigned char i = 0; i < 8; i++) {
   if (sensorValues[i] < DARKEST_READING) {
      return false;
   }
  }
  blinkLED();
  return true;
}

void rotate180(bool clockwise) {
  if (clockwise) {
    digitalWrite(right_dir_pin,HIGH);
    digitalWrite(left_dir_pin,LOW);
  } else {
    digitalWrite(right_dir_pin,LOW);
    digitalWrite(left_dir_pin,HIGH);
  }

  analogWrite(left_pwm_pin,50);
  analogWrite(right_pwm_pin,50);

  delay(2000);
}

void stopMotors() {
  analogWrite(left_pwm_pin,0);
  analogWrite(right_pwm_pin,0);
  delay(1000);
}

void loop() {

  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();

  if (hasDetectedCrossbar()) {
   // rotate180(true);
  }
  stopMotors();
    
  }
