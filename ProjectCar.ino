#include <ECE3.h>

//encoders

uint16_t sensorValues[8]; 

uint16_t sensorMins[8] = {600,  528, 552, 552, 460, 529, 575, 598};//{573, 504, 527, 573, 504, 550, 596, 596}; //may 6 -- get sensorMins each time?
uint16_t sensorMaxes[8] = {1900, 1972,  1948,  1948,  2040,  1971,  1925,  1902};//{1927, 1996,  1973,  1927,  1996,  1950,  1904,  1904}; // get sensorMaxes each time?


//const int DARKEST_READING = 1000;

const int left_nslp_pin=31; 
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; 
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int sw_1_pin = 73;
const int sw_2_pin = 74;

const int LED_RF = 41; //right front - back left yellow
const int LED_LF = 51; //left front - back right yellow
const int LED_RB = 58; //right back - left front red
const int LED_LB = 57; //left back - right front red

//MOTOR CONSTANTS
const float Kp = 0.1; //250 error = 25 PWM change
const float Kd = 0.0;
const float Kf = 11.0; // out of 255
float P;
float D;
float F;

float error = 0.0;
float prev_error = error;

int output = 0;
int currentLSpeed = 0;
int currentRSpeed = 0;
int lOutput = 0;
int rOutput = 0;
const float errorThreshold = 0;//5; //mm

int carState;
const int STOP = 0;
const int RUN = 1;
const int DONUT = 2;
const int FIND_PATH = 3;

const float MAX_SPEED = 255;
const float MAX_SPEED_INVERSE = 1 / MAX_SPEED;

void setup() {
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,HIGH);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin, HIGH);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(sw_1_pin,INPUT);
  pinMode(sw_2_pin,INPUT);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();

  Serial.begin(9600); 
  delay(2000); //like IR sensor example 

  calibrate();

  carState = 1;

  //set to coast? or brake is better for control?
  
}

void loop() {

  ECE3_read_IR(sensorValues);
  calculateError();

  P = Kp * error;
  D = Kd * (error - prev_error);
  F = Kf;

  output = P + D;

   lOutput = F;
   rOutput = F;

  lOutput += output;
  rOutput -= output;

//    if (lOutput > 0) { //...doesn't work?
//    digitalWrite(left_dir_pin,HIGH);
//  } else {
//    digitalWrite(left_dir_pin,LOW);
//  }
//
//    if (rOutput > 0) {
//    digitalWrite(right_dir_pin,HIGH);
//  } else {
//    digitalWrite(right_dir_pin,LOW);
//  }


Serial.print(error); 
Serial.println();
  //Serial.print("\t");
  //.Serial.println(rOutput);

 // analogWrite(left_pwm_pin, lOutput);
 // analogWrite(right_pwm_pin, rOutput);

  //delay(100);
        
//  Serial.print("CAR STATE: ");
//  Serial.print(carState);
//  switch (carState) {
//    case STOP:
//      stopMotors();
//      if (digitalRead(sw_1_pin)) {
//        carState = RUN;
//      }
//      break;
//    case RUN:
      // read raw sensor values
//        ECE3_read_IR(sensorValues);
//        calculateError();
//       // Serial.println(error);
//        runPID();
        // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
        // 2500 means minimum reflectance
//        for (unsigned char i = 0; i < 8; i++)
//        {
//          Serial.print(sensorValues[i]);
//          Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//        }
//        Serial.println();
      
//        if (hasDetectedCrossbar()) {
//         // rotate180(true);
//        }
//      
//        if (hasLostPath()) {
//         // carState = STOP;
//        }
//        
//      break;
//    case DONUT:
//      break;
//    case FIND_PATH:
//      break;
//  }


    
  }

//sensor fusion
void calculateError() {

  for (unsigned char i = 0; i < 8; i++) {
    sensorValues[i] -= sensorMins[i]; //644  573 596 620 549 573 573 596 -- mins

    sensorValues[i] = sensorValues[i] / sensorMaxes[i] * 1000;
  }
  
  error = sensorFusionAlg();
}

void runPID() {
  calculateError();
  //Serial.println(error);
  P = Kp * error;
  D = Kd * (error - prev_error);
  F = Kf;

//  if (abs(error) < errorThreshold) {
//    return; //skip the rest of this and just don't do anything
//  }

  output = P + D;
//
  lOutput = output;
  rOutput = output;

  lOutput += F;
  rOutput += F;
//
//  if (lOutput > MAX_SPEED) {
//    lOutput = MAX_SPEED;
//  }
//
//  if (rOutput > MAX_SPEED) {
//    rOutput = MAX_SPEED;
//  }
//
//  if (lOutput > 0) {
//    digitalWrite(left_dir_pin,LOW);
//  } else {
//    digitalWrite(left_dir_pin,HIGH);
//  }
//
//    if (rOutput > 0) {
//    digitalWrite(right_dir_pin,LOW);
//  } else {
//    digitalWrite(right_dir_pin,HIGH);
//  }

  //Serial.print("L: ");
 // Serial.print(lOutput);
 //Serial.print('\t');
 // Serial.print("R: ");
  Serial.println(rOutput);
  //analogWrite(left_pwm_pin, lOutput);
  //analogWrite(right_pwm_pin, rOutput);

//4 cases

  prev_error = error;
}

float sensorFusionAlg() {
  return (-8.0*sensorValues[0]-4.0*sensorValues[1]-2.0*sensorValues[2]-1.0*sensorValues[3]+1.0*sensorValues[4]+2.0*sensorValues[5]+4.0*sensorValues[6]+8.0*sensorValues[7])/4.0;
}


bool hasLostPath() { //if all sensors read <500, we lost the path
   for (unsigned char i = 0; i < 8; i++) {
    if (sensorValues[i] < 600) {
      return false;
    }
  }
  blinkLED();
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
   if (sensorValues[i] > 1800) {
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
