//DIFFERENTIAL STEERING
//(C) Rein Velt (rein@velt.org)
//SMALL PIECE OF FIRMWARE TO READ A RC RECEIVER (THROTTLE+STEERING)
//AND SEND A PWM SIGNAL TO 2 DIFFERENT MOTORS
//(DIFFERENTIAL STEERING)


//forward/reverse pins per motor
int pinDirectionMotorL = 7; //dir1
int pinDirectionMotorR = 4; //dir2

//output ports (PWM) for throttle/pwm/speedcontrol per motor
int pinPwmMotorL = 6;
int pinPwmMotorR = 5;

//input ports (for the rc receiver)
int pinInputThrottle = A0;
int pinInputSteering = A1;


void setup() {

  TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz (Output pins 5,6)
  pinMode(pinInputThrottle, INPUT);
  pinMode(pinInputSteering, INPUT);
  pinMode(pinDirectionMotorL, OUTPUT);
  pinMode(pinDirectionMotorR, OUTPUT);
  pinMode(pinPwmMotorL, OUTPUT);
  pinMode(pinPwmMotorR, OUTPUT);
  analogWrite(pinPwmMotorL,0);
  analogWrite(pinPwmMotorR,0);
  Serial.begin(115200);
  delay(500);
  Serial.println("#DIFFERENTIAL-STEERING-CONTROLLER");
  Serial.println("MOTOR-L MOTOR-R");
}

void setMotors(int throttle, int steering)
{
  int dirReverse = 0;
  if (throttle < 0) {
    dirReverse = 1;
  }
  int powerMixL = min(255, abs(map(steering, 255, -255, 0, throttle * 2)));
  int powerMixR = min(255, abs(map(steering, -255, 255, 0, throttle * 2)));

  //drive the motor pins
  digitalWrite(pinDirectionMotorL, dirReverse);
  digitalWrite(pinDirectionMotorR, dirReverse);
  analogWrite(pinPwmMotorL, powerMixL);
  analogWrite(pinPwmMotorR, powerMixR);

  //do some debugging to the serial port
  Serial.print(powerMixL);
  Serial.print("\t");
  Serial.print(powerMixR);
  Serial.print("\t");
}

int limitize(int value)
{
  if (value > -10 && value < 10)
  {
    value = 0;
  }
  value = min(value, 255);
  value = max(value, -255);
  return value;
}

void loop() {

  int inputThrottle = pulseIn(pinInputThrottle, HIGH, 35000);
  int inputSteering = pulseIn(pinInputSteering, HIGH, 35000);

  //the numbers can be a bit different per receiver
  int mappedThrottle = map(inputThrottle, 930, 1950, -255, 255); //map to -255 (fast reverse)..0(stop)..+255(fast forward)
  int mappedSteering = map(inputSteering, 930, 1955, -255, 255); //map to -255 (left)..0(stop) ..+255(right)

  setMotors(limitize(mappedThrottle), limitize(mappedSteering));
  Serial.println();
  delay(50);
}
