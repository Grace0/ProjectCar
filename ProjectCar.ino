#include <ECE3.h>

uint16_t sensorValues[8]; 

const int DARKEST_READING = 1000;

const int left_nslp_pin=31; 
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; 
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int sw_1_pin = 73;
const int sw_2_pin = 74;

const int LED_RF = 41;

const float Kp = 0.0;
const float Kd = 0.0;
float P;
float D;

float error = 0.0;
float prev_error = error;

int motorOutput = 0;
const float errorThreshold = 5; //mm

int carState;
const int STOP = 0;
const int RUN = 1;

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

  pinMode(sw_1_pin,INPUT);
  pinMode(sw_2_pin,INPUT);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();

  Serial.begin(9600); 
  delay(2000); 

  calibrate();

  carState = 0;

  //set to coast? or brake is better for control?
  
}

void loop() {

  switch (carState) {
    case STOP:
      stopMotors();
      if (digitalRead(sw_1_pin)) {
        carState = RUN;
      }
      break;
    case RUN:
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
      
        if (hasLostPath()) {
          carState = STOP;
        }
      break;
  }


    
  }

bool hasLostPath() { //if all sensors read 2500, we lost the path
   for (unsigned char i = 0; i < 8; i++) {
    if (sensorValues[i] < 2500) {
      return false;
    }
  }
  return true;
}

bool calibrate() {
  bool isSuccess;
  isSuccess = true;
  return isSuccess;
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

//sensor fusion
void calculateError() {
  error = 0.0;
}

void runPID() {
  calculateError();
  P = Kp * error;
  D = Kd * (error - prev_error);

  if (abs(error) < errorThreshold) {
    return; //skip the rest of this and just don't do anything
  }

//4 cases
  if (error < 0) {
    digitalWrite(right_dir_pin,LOW);
    analogWrite(right_pwm_pin,50);
    digitalWrite(left_dir_pin,LOW);
    analogWrite(left_pwm_pin,50);
  } else {
    digitalWrite(left_dir_pin,LOW);
    analogWrite(left_pwm_pin,50);
    digitalWrite(right_dir_pin,LOW);
    analogWrite(right_pwm_pin,50);
  }
  
  prev_error = error;
}
