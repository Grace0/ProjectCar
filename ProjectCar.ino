#include <ECE3.h>

//encoders

uint16_t sensorValues[8]; 

int sensorMins[8] = {552, 504, 504, 504, 435, 481, 528, 552 };//{600,  528, 552, 552, 460, 529, 575, 598};//{573, 504, 527, 573, 504, 550, 596, 596}; old //may 6 -- get sensorMins each time? 
int sensorMaxes[8] = {1900, 1972,  1948,  1948,  2040,  1971,  1925,  1902};//}{1927, 1996,  1973,  1927,  1996,  1950,  1904,  1904}; // get sensorMaxes each time? 
int sensorWeight[8] = {-8, -4, -2, -1, 1, 2, 4, 8}; //{-15, -14, -12, -8, 8, 12, 14, 15}; // 

int carState;
const int RUN = 0;
const int DONUT = 1;
const int STOP = 2;
//const int FIND_PATH = 3;

const int left_nslp_pin=31; 
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; 
const int right_dir_pin=30;
const int right_pwm_pin=39;

//MOTOR CONSTANTS
const float Kp = 0.03; //250 error = 25 PWM change
const float Kd = 0.01;//0.002;
const int Kf = 40; 
int P = 0;
int D = 0;

int error = 0.0;
int prev_error = error;

int output = 0;

int lOutput = 0;
int rOutput = 0;

const int MAX_SPEED = 200;
bool hasTurnedAlready = false;
bool prevHasDetectedCrossbar = false;

const int LED_RF = 41;

void setup() {
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin,HIGH);

  
  ECE3_Init();
  carState = RUN;
 
  delay(3000); 

}

void loop() {

  ECE3_read_IR(sensorValues);
 
    switch (carState) {
    case RUN:
        error = 0;
        for (int i = 0; i < 8; i++) {
          error += sensorWeight[i] * ((-sensorMins[i] + sensorValues[i]) * 1000.0 / sensorMaxes[i]);
        }
      
       error = error / 4;
      
        P = Kp * error;
        D = Kd * (error - prev_error);
      
        output = P + D;
      
         lOutput = Kf;
         rOutput = Kf;
      
        lOutput -= output;
        rOutput += output;
      
          if (lOutput < 0) {
          lOutput = 0;
        }
      
        if (rOutput < 0) {
          rOutput = 0;
        }
      
        if (lOutput > MAX_SPEED) {
          lOutput = MAX_SPEED;
        }
      
        if (rOutput > MAX_SPEED) {
          rOutput = MAX_SPEED;
        }
      
        analogWrite(left_pwm_pin, lOutput);
        analogWrite(right_pwm_pin, rOutput);
        prev_error = error;

        if (hasDetectedCrossbar()) {
          if (!prevHasDetectedCrossbar) {
            prevHasDetectedCrossbar = true;
          } else { //second time
              if (!hasTurnedAlready) {
              carState = DONUT;
            } else {
              carState = STOP;
            }
            prevHasDetectedCrossbar = false; //reset
          }
        }
      break;
    case DONUT:
      rotate180(true);
      carState = RUN;
      break;
    case STOP:
      stopMotors();
      break; //stay stuck in this state
  }

  }

 void rotate180(bool clockwise) {
  if (clockwise) {
    digitalWrite(right_dir_pin,HIGH);
    digitalWrite(left_dir_pin,LOW);
  } 

  analogWrite(left_pwm_pin,50);
  analogWrite(right_pwm_pin,50);
  delay(1500);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(left_dir_pin,LOW);
  analogWrite(left_pwm_pin,50);
  analogWrite(right_pwm_pin,50);
  delay(100);

  hasTurnedAlready = true;
}

void stopMotors() {
  analogWrite(left_pwm_pin,0);
  analogWrite(right_pwm_pin,0);
}

bool hasDetectedCrossbar() {
  for (unsigned char i = 0; i < 8; i++) {
   if (sensorValues[i] < 1500) {
      return false;
   }
  }
  digitalWrite(LED_RF, HIGH);
  return true;
}
