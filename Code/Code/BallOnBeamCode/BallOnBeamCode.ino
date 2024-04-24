// Connection of absolute magnet encoder
// SDA auf analog pin 4 top one left side locking at motor 
// SCL auf analog pin 5 middle one left side locking at motor
// DIR auf digital pin 2 bottom one left side locking at motor


#include <Wire.h>
#include <AS5600.h>
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

#define PWM 9 // connect to ENA at motor controller
#define IN1 6 // connet to IN1 at motor controller
#define IN2 7 // connect to IN2 at motor controller
#define DELTA_T_BALL 0.09 // sampling time ball
#define INFRA_RED_SENSOR A0 // orange cable
#define MOTOR_CURRENT_BOUND 255
#define MOTOR_ANGLE_BOUND 90

AMS_5600 ams5600;

int print_counter = 0;

int angle_pref = 0;
int angle_sum = 0;
float gear_ratio = 46.8;
int start_angle = 0;

// PID motor controller
float kp = 22; // 22
float kd = 0.4; // 0.4
float ki = 0.01; // 2
float motor_test_target = -90;

long prevT = 0;
float eprev = 0;
float eintegral = 0;
float target_prev = 0;

// PID ball controller
float kp_ball = 10; // 10
float kd_ball = 500; // 500
float ki_ball = 0.0;
float setpoint_prec = 0;
float u_ball = 0;

float target_ball = 20; // in cm

long prevT_ball = 0;
float eprev_ball = 0;
float dedt_ball_prev = 0;
float eintegral_ball = 0;
float target_prev_ball = 0;

float alpha_ball = 0.008; // 0.01
float alpha_d_ball = 0.01; // 0.01

void setup() {
  Serial.begin(9600);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(INFRA_RED_SENSOR,INPUT);

  Wire.begin();
  if(ams5600.detectMagnet() == 0 ){
    while(1){
        if(ams5600.detectMagnet() == 1 ){
            SERIAL.print("Current Magnitude: ");
            SERIAL.println(ams5600.getMagnitude());
            break;
        }
        else{
            SERIAL.println("Can not detect magnet");
        }
        delay(1000);
    }
  }
  start_angle = convertRawAngleToDegrees(ams5600.getRawAngle());
  
}

void loop() 
{
  
  // time difference
  long currT = micros();
  float deltaT_motor = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
  float motor_target = ballPositionController(target_ball, DELTA_T_BALL);

  // Stellgrößebeschränkung
  if ( motor_target > MOTOR_ANGLE_BOUND)
  {
    motor_target = MOTOR_ANGLE_BOUND;
  }
  else if (motor_target < -MOTOR_ANGLE_BOUND)
  {
    motor_target = -MOTOR_ANGLE_BOUND;
  }
  
  motorControl(motor_test_target, deltaT_motor);
}

float ballPositionController(float target_ball, float deltaT)
{
  float ball_pos = get_dist(100);
  ball_pos = alpha_ball*ball_pos + (1-alpha_ball)*setpoint_prec; // filter ball pos
  setpoint_prec = ball_pos;
  
  // error
  float e_ball = target_ball - ball_pos;

  // derivative
  float dedt_ball = (e_ball-eprev_ball)/(deltaT);
  dedt_ball = alpha_d_ball*dedt_ball + (1 - alpha_d_ball)*dedt_ball_prev;    // filtering derivative

  // integral
  eintegral_ball = eintegral_ball + e_ball*deltaT;

  // control signal
  float u_ball = kp_ball*e_ball + kd_ball* dedt_ball + ki_ball*eintegral_ball;

  // store previous error
  eprev_ball = e_ball;
  dedt_ball_prev = dedt_ball;

  return u_ball;
}

float get_dist(int n)
{
  float distance_cm = 17569.7 * pow(analogRead(INFRA_RED_SENSOR), -1.2062);
  return distance_cm;
}

void motorControl(float target, float deltaT)
{
  // Read the position
  int pos = get_angle();
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;
  
  // motor power
  float pwr = fabs(u);
  if( pwr > MOTOR_CURRENT_BOUND ){
    pwr = MOTOR_CURRENT_BOUND;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  
  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  // store previous error
  eprev = e;
  target_prev = target;

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

  float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}

int get_angle()
{
  float angle = start_angle - convertRawAngleToDegrees(ams5600.getRawAngle());
  
  int angle_round = round(angle / gear_ratio);

  int compair = angle_pref - angle_round;
  if (abs(compair) > (200 / gear_ratio) && compair > 0)
  {
    angle_sum += round(360 / gear_ratio);
  }
  else if (abs(compair) > (200/ gear_ratio) && compair < 0)
  {
    angle_sum -= round(360 / gear_ratio);
  }
  if (angle_sum % 360 == 0)
  {
    angle_sum = 0;
  }
  angle_pref = angle_round;
    
  return angle_round + angle_sum;
}  

