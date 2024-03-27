#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>

const uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// PID constants
const double Kp_FORWARD = 2.4; // proportional
const double Ki_FORWARD = 3.5; // integral
const double Kd_FORWARD = 0.01; // derivative
const double Kp_TURN = 20; // proportional
const double Ki_TURN = 5; // integral
const double Kd_TURN = 0; // derivative
double Kp = 0;
double Ki = 0;
double Kd = 0;
const double PID_CHANGE_ANGLE = 10;

double targetAngle = 90;
// PID variables
double integral = 0;
double derivative;
double previous_error = 0;

const unsigned long REFRESHRATE = 50;

double leftSpeedAdjustment = 0;
double rightSpeedAdjustment = 0;

const int PIN_PING = 16;

enum State
{
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND
};
State curState;

// numbers represent the signature # stored in the Pixy for the corresponding marker
enum Direction
{
  LEFT, // pink
  RIGHT, // blue
  AROUND // green
};
Direction storedTurnDirection;

const int MOTOR_SPEED_F = 110; // range from -400 to 400
const int MOTOR_SPEED_B = 130; // range from -400 to 400
const float SPEED_OF_SOUND = 0.0343; // cm per microsecond

const float TURN_DISTANCE_THRESHOLD = 10;
const float REVERSE_START_DISTANCE_THRESHOLD = 5;
const float REVERSE_STOP_DISTANCE_THRESHOLD = 15;

DualMAX14870MotorShield motors;

Pixy2 pixy;

unsigned long turnTimer = 0;
const unsigned long TURN_HOLD_TIME = 500;

void setup() {
  Serial.begin(115200);

  if (!bno.begin())
  {
    Serial.println("BNO055 not detected!");
    while (1);
  }

  motors.enableDrivers();
  motors.flipM2(true);
  pixy.init();

  pinMode(3, OUTPUT);

  curState = FORWARD;

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  targetAngle = orientationData.orientation.x; // set target angle to currently facing angle
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double currentAngle = orientationData.orientation.x;

  unsigned long now = millis();
  double error = targetAngle - currentAngle;
  if (error > 180)
  {
    error -= 360;
  }
  else if (error < -180)
  {
    error += 360;
  }

  integral += error * (REFRESHRATE / 1000.0);
  if (abs(error - previous_error) >= 60)
  {
    derivative = 0;
  }
  else
  {
    derivative = (error - previous_error) / (REFRESHRATE / 1000.0);
  }

  double output = Kp * error + Ki * integral + Kd * derivative;
  //double output = Kp * error;
  previous_error = error;

  Serial.print("target (");
  Serial.print(targetAngle);
  Serial.print(") - current (");
  Serial.print(currentAngle);
  Serial.print(") = error (");
  Serial.print(error);
  Serial.print("), adjustment = (");
  Serial.print(output);
  Serial.print(", ");
  Serial.print(-output);
  Serial.println(")");

  adjustSpeed(output);

  float frontDistance = measureFrontDistance();
  if (frontDistance <= TURN_DISTANCE_THRESHOLD)
  {
    digitalWrite(3, HIGH);
  }
  else
  {
    digitalWrite(3, LOW);
  }

  switch (curState)
  {
    case FORWARD: // check for pixy
    {
      //Serial.println("go forward");
      
      setMotorSpeed(MOTOR_SPEED_F, MOTOR_SPEED_F);

      checkForTurnDirection();

      Kp = Kp_FORWARD;
      Ki = Ki_FORWARD;
      Kd = Kd_FORWARD;

      /*
      Serial.print("Distance - ");
      Serial.print(frontDistance);
      Serial.println(" cm");
      */
      
      if (frontDistance <= TURN_DISTANCE_THRESHOLD && frontDistance >= REVERSE_START_DISTANCE_THRESHOLD)
      {
        if (storedTurnDirection == LEFT)
        {
          curState = TURN_LEFT;
          changeTargetAngle(-90);
        }
        else if (storedTurnDirection == RIGHT)
        {
          curState = TURN_RIGHT;
          changeTargetAngle(90);
        }
        else if (storedTurnDirection == AROUND)
        {
          curState = TURN_AROUND;
          changeTargetAngle(180);
        }
      }
      if (frontDistance <= REVERSE_START_DISTANCE_THRESHOLD)
      {
        curState = BACKWARD;
      }

      break;
    }

    case BACKWARD:
    {
      //Serial.println("go backward");

      setMotorSpeed(-MOTOR_SPEED_B, -MOTOR_SPEED_B);
      
      if (frontDistance >= REVERSE_STOP_DISTANCE_THRESHOLD)
      {
        curState = FORWARD;
      }

      break;
    }

    case TURN_LEFT:
    {
      Serial.println("turn left");
      setMotorSpeed(0, 0);

      if (abs(error) <= PID_CHANGE_ANGLE)
      {
        Kp = Kp_TURN;
        Ki = Ki_TURN;
        Kd = Kd_TURN;
      }
      else
      {
        integral = 0;
        Kp = Kp_FORWARD;
        Ki = Ki_FORWARD;
        Kd = Kd_FORWARD;
      }

      if (aboutEquals(error, 0))
      { 
        if (now - turnTimer >= TURN_HOLD_TIME)
        {
          curState = FORWARD;
          storedTurnDirection = 0;
          integral = 0;
        }
      }
      else
      {
        turnTimer = millis();
      }
      break;
    }

    case TURN_RIGHT:
    {
      Serial.println("turn right");
      setMotorSpeed(0, 0);

      if (abs(error) <= PID_CHANGE_ANGLE)
      {
        Kp = Kp_TURN;
        Ki = Ki_TURN;
        Kd = Kd_TURN;
      }
      else
      {
        integral = 0;
        Kp = Kp_FORWARD;
        Ki = Ki_FORWARD;
        Kd = Kd_FORWARD;
      }

      Serial.println(now - turnTimer);
      if (aboutEquals(error, 0))
      {
        if (now - turnTimer >= TURN_HOLD_TIME)
        {
          curState = FORWARD;
          storedTurnDirection = 0;
          integral = 0;
        }
      }
      else
      {
        turnTimer = millis();
      }
      break;
    }

    case TURN_AROUND:
    {
      //Serial.println("turn around");
      setMotorSpeed(0, 0);

      if (abs(error) <= PID_CHANGE_ANGLE)
      {
        Kp = Kp_TURN;
        Ki = Ki_TURN;
        Kd = Kd_TURN;
      }
      else
      {
        integral = 0;
        Kp = Kp_FORWARD;
        Ki = Ki_FORWARD;
        Kd = Kd_FORWARD;
      }

      if (aboutEquals(error, 0))
      {
        if (now - turnTimer >= TURN_HOLD_TIME)
        {
          curState = FORWARD;
          storedTurnDirection = 0;
          integral = 0;
        }
      }
      else
      {
        turnTimer = millis();
      }
      break;
    }
  }

  delay(REFRESHRATE);
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  motors.setM2Speed(leftSpeed + leftSpeedAdjustment);
  motors.setM1Speed(rightSpeed + rightSpeedAdjustment);
}

/*
 * Checks recognized objects from the Pixy2
 * If the object is a stored color, update storedTurnDirection accordingly
 * return true if an object was read by the Pixy
 */
bool checkForTurnDirection()
{
  //Serial.println("check for turn direction");
  bool didFindMarker = false;
  
  pixy.ccc.getBlocks(false, 7, 1);
  if (pixy.ccc.numBlocks)
  {
    didFindMarker = true;
    
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    
    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
      Serial.print("   block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }

    uint16_t signature = pixy.ccc.blocks[0].m_signature;
    switch(signature)
    {
      case 1:
        storedTurnDirection = LEFT;
        break;

      case 2:
        storedTurnDirection = RIGHT;
        break;

      case 3:
        storedTurnDirection = AROUND;
        break;
    }
  }

  return didFindMarker;
}

/*
 * Uses the front PING sensor to measure the distance to the wall in front
 * Returns the distance in cm
 */
float measureFrontDistance()
{
  // clear signal pin
  pinMode(PIN_PING, OUTPUT);
  digitalWrite(PIN_PING, LOW);
  delayMicroseconds(5);

  // trigger sensor with pulse out
  digitalWrite(PIN_PING, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_PING, LOW);

  // measure pulse duration in (2-way travel time)
  pinMode(PIN_PING, INPUT);
  unsigned long pulseDuration = pulseIn(PIN_PING, HIGH);
  // convert pulse duration to physical distance
  float distance = (pulseDuration / 2) * SPEED_OF_SOUND;

  return distance;
}

void adjustSpeed(double output)
{
  leftSpeedAdjustment = output;
  rightSpeedAdjustment = -output;
}

void changeTargetAngle(double turn)
{
  targetAngle = targetAngle + turn;
  if (targetAngle >= 360)
  {
    targetAngle -= 360;
  }
  if (targetAngle <= 0)
  {
    targetAngle += 360;
  }
}

bool aboutEquals(float test, float target)
{
  return abs(target - test) <= 1;
}
