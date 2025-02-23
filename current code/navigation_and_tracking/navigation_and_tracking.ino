#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include "FireTimer.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <math.h>

Pixy2 pixy;
DualMAX14870MotorShield motors;

/*
* handles ZigBee communication
* also receives and stores current match data
*/
class MatchData
{
  bool status = 0; // true = match in progress
  int time = 0; // seconds
  int x = 0; // cm
  int y = 0; // cm

  bool isWaitingForUpdate = false;

  /*
  * read in the match data as a String from the ZigBee module
  * returns: String containing unparsed match data if available,
  *   empty String if data is not available
  */
  String receiveData()
  {
    String RX = ""; // format: M,TTTT,XXX,YYY

    int length = 0;
    bool isFinished = true;

    // first check if there's any data available at all before continuing
    if (Serial1.available())
    {
      isFinished = false;
    }

    // continue concatenating all data to the output string
    // THIS ASSUMES THE DATA IS FORMATTED AND BEING RECEIVED PROPERLY
    while (!isFinished)
    {
      if (Serial1.available())
      {
        char incoming = Serial1.read();
        RX.concat(incoming);
        length++;
      }
      
      if (length == 14)
      {
        isFinished = true;
      }
    }
    
    return RX;
  }

  void parseAndUpdate(String data)
  {
    String str_status = data.substring(0, 1);
    String str_time = data.substring(2, 6);
    String str_x = data.substring(7, 10);
    String str_y = data.substring(11, 14);

    // if the new data isn't complete, fall back on previous data
    if (str_status.equals("?") || str_status.equals("/")
    || str_time.equals("????") || str_time.equals("////")
    || str_x.equals("---") || str_y.equals("---"))
    {
      return;
    }
    // new data is complete, go ahead with parsing

    bool isValid = true;

    bool status_new = str_status.equals("1");
    int time_new = str_time.toInt();
    int x_new = str_x.toInt();
    int y_new = str_y.toInt();

    // if the new data isn't valid for some reason, fall back on previous data
    // otherwise, go ahead and update data
    if (time_new < 0 || x_new < 0 || y_new < 0)
    {
      return;
    }

    status = status_new;
    time = time_new;
    x = x_new;
    y = y_new;
  }

public:
  bool getStatus() { return status; }
  int getTime() { return time; }
  int getX() { return x; }
  int getY() { return y; }

  void setup()
  {
    Serial1.begin(115200); // communicating w/ ZigBee (for match data)
  }

  void update()
  {
    // if the ZigBee hasn't been asked for an update yet, do that and return
    if (!isWaitingForUpdate)
    {
      Serial1.print('?');
      isWaitingForUpdate = true;
      return;
    }
    // if we are waiting on the ZigBee module for an update, proceed

    // check on incoming data from ZigBee module
    String incoming = receiveData();
    // if data actually received, parse it and update
    if (!incoming.equals(""))
    {
      //Serial.print("received data: ");
      //Serial.println(incoming);
      parseAndUpdate(incoming);
      //print();
      // go back to requesting a new update
      isWaitingForUpdate = false;
    }
  }

  void print()
  {
    Serial.println("-----");

    Serial.print("Match in progress: ");
    Serial.println(status ? "YES" : "NO");

    Serial.print("Match time: ");
    Serial.print(time);
    Serial.println(" s");

    Serial.print("Robot position: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");

    Serial.println("-----");
  }
};
MatchData match;

/*
* object that acts as an independent PID
* takes in values for the coefficients, a clamp for the output, and a clamp for the integral component
* the compute function takes in the input and setpoint, as well as updates the output directly by reference
*/
class PID
{
  const float Kp;
  const float Ki;
  const float Kd;

  const float iWindupMin;
  const float iWindupMax;
  //const float iErrorMin;
  //const float iErrorMax;

  const float outputMin;
  const float outputMax;

  const unsigned int samplePeriod_ms;

  float i = 0;

  float curError = 0;
  float lastError = 0;

  FireTimer timer;

  float doCompute(float input, float setpoint, ulong timeDiff)
  {
    lastError = curError;
    curError = setpoint - input;

    float p = Kp * curError;
    float d = Kd * (curError - lastError);
    float iTemp = 0;
    //if (curError >= iErrorMin && curError <= iErrorMax)
    //{
      iTemp = i + Ki * (curError + lastError) / 2.0;
    //}
    iTemp = constrain(iTemp, iWindupMin, iWindupMax);

    float out = p + d;
    /* part of code i didn't rly understand from the PID explanation i found -- leaving it out for now
    float iMax = constrain(outputMax - out, 0, outputMax);
    float iMin = constrain(outputMin - out, outputMin, 0);
    i = constrain(iTemp, iMin, iMax);
    */
    i = iTemp;

    out += i;
    return constrain(out, outputMin, outputMax);
  }

public:
  PID(float K_proportional, float K_integral, float K_derivative, float outputMin = -1000, float outputMax = 1000, float integralWindupMin = -1000, float integralWindupMax = 1000, unsigned int samplePeriod_ms = 50)
  : Kp(K_proportional), Ki(K_integral), Kd(K_derivative), outputMin(outputMin), outputMax(outputMax), iWindupMin(integralWindupMin), iWindupMax(integralWindupMax), samplePeriod_ms(samplePeriod_ms)
  {
    timer.begin(samplePeriod_ms);
  }
  
  /*
  // switched to an approach where we literally constrain the integral term to only be calculated within a certain error range rather than clamp the integral term
  // this is more like the approach my group took to lab 3/4 (? can't remember which one) so might be easier to tune for me
  PID(float K_proportional, float K_integral, float K_derivative, float outputMin = -1000, float outputMax = 1000, float integralErrorMin = -1000, float integralErrorMax = 1000, unsigned int samplePeriod_ms = 50)
  : Kp(K_proportional), Ki(K_integral), Kd(K_derivative), outputMin(outputMin), outputMax(outputMax), iErrorMin(integralErrorMin), iErrorMax(integralErrorMax), samplePeriod_ms(samplePeriod_ms)
  {
    timer.begin(samplePeriod_ms);
  }
  */

  void start()
  {
    reset();

    timer.start();
  }

  void reset()
  {
    curError = 0;
    lastError = 0;
    i = 0;
  }

  void compute(float input, float setpoint, float &output)
  {
    if (timer.fire())
    {
      output = doCompute(input, setpoint, timer.timeDiff);
    }
  }
};

/*
* stores position and orientation data
* these will be computed relatively based on match data (for position) and the IMU (for orientation)
* this will also handle determining which zone the robot is in, as well as moving to a target position
*/
class Movement
{
  const float RAD2DEG = 180 / 3.14159;

  const int MOTOR_SPEED_F = 150; // range from -400 to 400

  const unsigned long STOP_PERIOD = 400; // MS to wait between hitting target position and swinging to target angle
  unsigned long stopTime = 0;

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

  float baseAngle = 0;

  float angle = 0;
  int x = 0;
  int y = 0;

  enum {
    NONE,
    TARGET_SET,
    MOVE_TO_POS,
    STOP,
    MOVE_TO_ANGLE
  } targetState;
  int targetX = 0;
  int targetY = 0;
  float targetAngle = 0;
  float targetAngleIntermediate = 0;

  float turnAdjust = 0;
  float speedAdjust = 0;

  //PID pid_adjustTurning = PID(2.4, 3.5, 0.01, -400, 400);
  PID pid_adjustTurning = PID(10, 0, 20, -200, 200);
  PID pid_pureTurning = PID(10, 10, 20, -200, 200, -75, 75);
  //PID pid_pureTurning = PID(10, 10, 20, -400, 400, -50, 50);

  PID pid_movement = PID(5, 0, 5, 0, MOTOR_SPEED_F);

  unsigned long abeTimePos = 0;
  unsigned long abePrevTimePos = 0;
  const unsigned long ABOUTEQUALS_TIMER_POSITION = 100; // time in MS position should be within the threshold before returning true
  const float ABOUTEQUALS_RANGE_POSITION = pow(3, 2); // square distance in CM(^2)
  bool aboutEqualsPosition(float testX, float testY, float targetX, float targetY)
  {
    // check if in range using square distance (sqrt is slow, idk how relevant that is tho for this application tbh)
    bool isInRange = squareDistanceBetween(testX, testY, targetX, targetY) <= ABOUTEQUALS_RANGE_POSITION;

    unsigned long curTime = millis();
    if (isInRange) // still within range, just check how long it's been
    {
      abeTimePos += curTime - abePrevTimePos;

      if (abeTimePos >= ABOUTEQUALS_TIMER_POSITION) // sufficient time within range
      {
        abeTimePos = 0;
        return true;
      }
    }
    else // out of range now, reset the timer
    {
      abeTimePos = 0;
    }

    abePrevTimePos = curTime;
    return false;
  }

  unsigned long abeTimeAngle = 0;
  unsigned long abePrevTimeAngle = 0;
  const unsigned long ABOUTEQUALS_TIMER_ANGLE = 100; // time in MS angle should be within the threshold before returning true
  const float ABOUTEQUALS_RANGE_ANGLE = 1.5; // angle in degrees -> [target - range, target + range]
  bool aboutEqualsAngle(float testAngle, float targetAngle)
  {
    // check if in range using square distance (sqrt is slow, idk how relevant that is tho for this application tbh)
    bool isInRange = abs(targetAngle - testAngle) <= ABOUTEQUALS_RANGE_ANGLE;

    unsigned long curTime = millis();
    if (isInRange) // still within range, just check how long it's been
    {
      abeTimeAngle += curTime - abePrevTimeAngle;

      if (abeTimeAngle >= ABOUTEQUALS_TIMER_ANGLE) // sufficient time within range
      {
        abeTimeAngle = 0;
        return true;
      }
    }
    else // out of range now, reset the timer
    {
      abeTimeAngle = 0;
    }

    abePrevTimeAngle = curTime;
    return false;
  }

  float angleError(float input, float setpoint)
  {
    float e = setpoint - input;
    if (e > 180)
    {
      e -= 360;
    }
    else if (e < -180)
    {
      e += 360;
    }

    return e;
  }

  float wrapAngle(float input)
  {
    return input - 360 * floor(input/360);
  }

  float angleBetween(float x1, float y1, float x2, float y2)
  {
    return wrapAngle(atan2f(y2 - y1, x2 - x1) * RAD2DEG);
  }

  float squareDistanceBetween(float x1, float y1, float x2, float y2)
  {
    return pow(y2 - y1, 2) + pow(x2 - x1, 2);
  }

public:
  void setup()
  {
    if (!bno.begin())
    {
      Serial.println("BNO055 IMU not detected!");
      while (1);
    }

    pid_adjustTurning.start();
    pid_pureTurning.start();

    pid_movement.start();

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    baseAngle = orientationData.orientation.x;

    unsigned long curTime = millis();
    abeTimePos = curTime;
    abeTimeAngle = curTime;
  }

  void print()
  {
    Serial.print("MOVEMENT: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(") | ");
    Serial.println(angle);
  }


  unsigned long tempTimer = 0;
  void update(int x, int y)
  {
    this->x = x;
    this->y = y;

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    // offset the angle by the base angle and wrap to [0, 360)
    angle = wrapAngle(-orientationData.orientation.x + baseAngle);

    
    if (targetState == NONE)
    {
      Serial.println("---- NO TARGET");
    }
    else
    {
      Serial.print("==== TARGET: (");
      Serial.print(targetX);
      Serial.print(", ");
      Serial.print(targetY);
      Serial.print(") | ");
      Serial.print(targetAngleIntermediate);
      Serial.print(", ");
      Serial.println(targetAngle);
    }

    switch (targetState)
    {
      // initial target has been set:
      // move to the orientation required to go forward to the correct position
      case TARGET_SET:
      {
        Serial.println("TARGET SET:");
        pid_pureTurning.compute(-angleError(angle, targetAngleIntermediate), 0, turnAdjust);
        motors.setM1Speed(0 + turnAdjust);
        motors.setM2Speed(0 - turnAdjust);
        Serial.print("\tangle: ");
        Serial.print(angle);
        Serial.print(" | target: ");
        Serial.print(targetAngleIntermediate);
        Serial.print(" | error: ");
        Serial.println(angleError(angle, targetAngleIntermediate));
        Serial.print("\tturnAdjust: ");
        Serial.println(turnAdjust);

        if (aboutEqualsAngle(angle, targetAngleIntermediate))
        {
          targetState = MOVE_TO_POS;
          pid_adjustTurning.reset();
          pid_movement.reset();

          tempTimer = millis();
        }
        break;
      }

      // we're facing the correct position now:
      // move to the correct position
      case MOVE_TO_POS:
      {
        Serial.println("MOVE TO POS:");

        targetAngleIntermediate = angleBetween(x, y, targetX, targetY);
        float squareDist = squareDistanceBetween(x, y, targetX, targetY);
        
        pid_adjustTurning.compute(-angleError(angle, targetAngleIntermediate), 0, turnAdjust);
        pid_movement.compute(-squareDist, 0, speedAdjust);
        motors.setM1Speed(speedAdjust + turnAdjust);
        motors.setM2Speed(speedAdjust - turnAdjust);
        Serial.print("\tangle: ");
        Serial.print(angle);
        Serial.print(" | target: ");
        Serial.print(targetAngleIntermediate);
        Serial.print(" | error: ");
        Serial.println(angleError(angle, targetAngleIntermediate));
        Serial.print("\tspeedAdjust: ");
        Serial.println(turnAdjust);
        Serial.print("\tposition: (");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print(") | target: (");
        Serial.print(targetX);
        Serial.print(", ");
        Serial.print(targetY);
        Serial.print(") | square distance: ");
        Serial.println(squareDist);
        
        
        if (aboutEqualsPosition(x, y, targetX, targetY))
        {
          stopTime = millis();
          targetState = STOP;
        }
        
        /*
        unsigned long curTime = millis();
        if (curTime - tempTimer >= 2000) // wait time since we don't have position checking up yet
        {
          stopTime = curTime;
          targetState = STOP;
        }
        */
        
        break;
      }

      case STOP:
      {
        Serial.println("BRIEF PAUSE TO SETTLE");
        motors.setM1Speed(0);
        motors.setM2Speed(0);

        unsigned long curTime = millis();
        if (curTime - stopTime >= STOP_PERIOD)
        {
          // no target angle set, we're done here
          if (targetAngle < 0)
          {
            targetState = NONE;
            turnAdjust = 0;
            motors.setM1Speed(0);
            motors.setM2Speed(0);
          }
          else
          {
            targetState = MOVE_TO_ANGLE;
            pid_pureTurning.reset();
          }
        }
        break;
      }

      // we're in the correct position now:
      // move to the correct orientation (if there is a target angle set)
      case MOVE_TO_ANGLE:
      {
        Serial.println("MOVE TO ANGLE:");
        pid_pureTurning.compute(-angleError(angle, targetAngle), 0, turnAdjust);
        motors.setM1Speed(0 + turnAdjust);
        motors.setM2Speed(0 - turnAdjust);
        Serial.print("\tangle: ");
        Serial.print(angle);
        Serial.print(" | target: ");
        Serial.print(targetAngle);
        Serial.print(" | error: ");
        Serial.println(angleError(angle, targetAngle));

        if (aboutEqualsAngle(angle, targetAngle))
        {
          targetState = NONE;
          turnAdjust = 0;
          motors.setM1Speed(0);
          motors.setM2Speed(0);
        }
        break;
      }

      // else (case NONE) do nothing
    }


    /* CODE ONLY USED FOR TURNING PID TEST
    Serial.println("TARGET SET:");
    pid_pureTurning.compute(-angleError(angle, targetAngle), 0, speedAdjust);
    motors.setM1Speed(0 + speedAdjust);
    motors.setM2Speed(0 - speedAdjust);
    Serial.print("\tangle: ");
    Serial.print(angle);
    Serial.print(" | target: ");
    Serial.print(targetAngle);
    Serial.print(" | error: ");
    Serial.println(angleError(angle, targetAngle));
    Serial.print("\tspeedAdjust: ");
    Serial.println(speedAdjust);
    //*/

    /* CODE ONLY USED FOR MOVING PID TEST
    Serial.println("FORWARD MOVEMENT SET:");
    targetAngle = 0;
    pid_adjustMovement.compute(-angleError(angle, targetAngle), 0, speedAdjust);
    motors.setM1Speed(MOTOR_SPEED_F + speedAdjust);
    motors.setM2Speed(MOTOR_SPEED_F - speedAdjust);
    Serial.print("\tangle: ");
    Serial.print(angle);
    Serial.print(" | target: ");
    Serial.print(targetAngle);
    Serial.print(" | error: ");
    Serial.println(angleError(angle, targetAngle));
    Serial.print("\tspeedAdjust: ");
    Serial.println(speedAdjust);
    //*/
  }

  /*
  void setTargetAngle(int targetAngle)
  {
    this->targetAngle = targetAngle;
  }
  */

  void setTarget(int targetX, int targetY, int targetAngle)
  {
    targetState = TARGET_SET;
    this->targetX = targetX;
    this->targetY = targetY;
    this->targetAngle = targetAngle;

    if (aboutEqualsPosition(x, y, targetX, targetY))
    {
      // no target angle set, we're done here
      if (targetAngle < 0)
      {
        targetState = NONE;
        turnAdjust = 0;
        motors.setM1Speed(0);
        motors.setM2Speed(0);
      }
      else
      {
        targetState = MOVE_TO_ANGLE;
        pid_pureTurning.reset();
      }
    }

    /*
    //float t = atan2f(x - targetX, targetY - y) * RAD2DEG;
    float t = atan2f(targetY - y, targetX - x) * RAD2DEG;
    targetAngleIntermediate = wrapAngle(t);
    */
    targetAngleIntermediate = angleBetween(x, y, targetX, targetY);
    turnAdjust = 0;
    pid_pureTurning.reset();
  }

  bool isTargeting()
  {
    return targetState != NONE;
  }
};
Movement movement;

// number = bitmap to select for that signature (1 byte, 128-place specifies color codes)
enum PixySignature
{
  PUCK = 1, // orange, 
  GOAL1 = 2, // red
  GOAL2 = 4 // pink
};

struct ScreenPos
{
  int x;
  int y;

  ScreenPos(int x = -1, int y = -1) : x(x), y(y)
  { }

  ScreenPos& operator=(const ScreenPos& other)
  {
    x = other.x;
    y = other.y;
    return *this;
  }

  bool operator==(const ScreenPos& other) const
  {
    return (x == other.x && y == other.y);
  }
};
ScreenPos SCREEN_POS_CENTER(315/2, 207/2);
ScreenPos SCREEN_POS_NULL(-1, -1);

PID pid_puckTracker = PID(2, 0, 0, -400, 400);

ScreenPos screenPosPuck;
ScreenPos screenPosGoal1;
ScreenPos screenPosGoal2;

float speedAdjust = 0;

unsigned long waitTimer = 0;
const unsigned long TIME_TO_WAIT = 1000;
void setup()
{
  Serial.begin(115200); // console output

  delay(2000);

  match.setup();
  movement.setup();
  
  //pixy.init();
  motors.enableDrivers();
  motors.flipM2(true); // flipped so both go forward with positive speeds

  //puckTracker.start();
  //movement.setTarget(10, 10, 45);
  //movement.setTargetAngle(45);
  waitTimer = millis();
}

int i = -1;
bool b = true;
int targets[][3] = {
  {35, 35, 45}, // red right corner
  {35, 90, 315}, // red left corner
  {195, 90, 225}, // pink right corner
  {195, 35, 135}, // pink left corner
  {120, 60, 0}, // center, facing pink
  {120, 60, 180} // center, facing red
  //{10, 10, 90},
  //{10, -10, 0},
  //{-10, 0, 270}
};
int targets_len = 6;

void loop()
{
  // fetch latest match data from ZigBee
  match.update();
  movement.update(match.getX(), match.getY());

  if (millis() - waitTimer < TIME_TO_WAIT)
  {
    return;
  }

  //match.print();
  //movement.print();

  if (b)
  {
    if (!movement.isTargeting())
    {
      b = false;
      i++;
    }
  }

  if (!b && i < targets_len)
  {
    movement.setTarget(targets[i][0], targets[i][1], targets[i][2]);
    b = true;
  }

  /*
  screenPosPuck = pixyScan(PixySignature::PUCK, 3);
  Serial.print("Puck Screen Pos: (");
  Serial.print(screenPosPuck.x);
  Serial.print(", ");
  Serial.print(screenPosPuck.y);
  Serial.print(")");
  if (screenPosPuck == SCREEN_POS_NULL)
  {
    speedAdjust = 0;
    Serial.println(" => not on screen");
  }
  else
  {
    puckTracker.compute(screenPosPuck.x, SCREEN_POS_CENTER.x, speedAdjust);
    Serial.print(" => ");
    Serial.println(speedAdjust);
  }

  motors.setM1Speed(0 + speedAdjust);
  motors.setM2Speed(0 - speedAdjust);
  */

  /*
  screenPosGoal1 = pixyScan(PixySignature::GOAL1, 1);
  Serial.print("Goal 1 Screen Pos: (");
  Serial.print(screenPosGoal1.x);
  Serial.print(", ");
  Serial.print(screenPosGoal1.y);
  Serial.print(") => error: ");
  if (screenPosGoal1 == SCREEN_POS_NULL)
  {
    Serial.println("not on screen");
  }
  else
  {
    Serial.println(SCREEN_POS_CENTER.x - screenPosGoal1.x);
  }

  screenPosGoal2 = pixyScan(PixySignature::GOAL2, 1);
  Serial.print("Goal 1 Screen Pos: (");
  Serial.print(screenPosGoal2.x);
  Serial.print(", ");
  Serial.print(screenPosGoal2.y);
  Serial.print(") => error: ");
  if (screenPosGoal2 == SCREEN_POS_NULL)
  {
    Serial.println("not on screen");
  }
  else
  {
    Serial.println(SCREEN_POS_CENTER.x - screenPosGoal2.x);
  }
  Serial.println(""); 
  */

  delay(50);
}

ScreenPos pixyScan(PixySignature signatureBitmap, int maxBlocks)
{
  /*
  false -> don't wait for block data to continue
  PixySignature -> specifies which signature to select
  int -> max blocks to select (sorted by size)
  */
  uint8_t numBlocks = pixy.ccc.getBlocks(false, signatureBitmap, maxBlocks);

  // didn't find any objects, break out of the method immediately
  if (pixy.ccc.blocks <= 0)
  {
    return SCREEN_POS_NULL;
  }

  int xsum = 0;
  int ysum = 0;
  for (uint8_t i = 0; i < numBlocks; i++)
  {
    Block curBlock = pixy.ccc.blocks[i];
    xsum += curBlock.m_x;
    ysum += curBlock.m_y;
  }

  return ScreenPos(xsum / numBlocks, ysum / numBlocks);
}