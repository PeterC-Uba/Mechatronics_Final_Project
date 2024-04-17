#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include "FireTimer.h"

Pixy2 pixy;
DualMAX14870MotorShield motors;

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

class PID
{
  const float Kp;
  const float Ki;
  const float Kd;

  const float iWindupMin;
  const float iWindupMax;

  const float outputMin;
  const float outputMax;

  const unsigned int samplePeriod_ms;

  float setpoint = 0;
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
    float iTemp = i + Ki * (curError + lastError) / 2.0;
    iTemp = constrain(iTemp, iWindupMin, iWindupMax);

    float out = p + d;
    /*
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

  void start()
  {
    setpoint = 0;

    curError = 0;
    lastError = 0;

    timer.start();
  }

  void compute(float input, float setpoint, float &output)
  {
    if (timer.fire())
    {
      output = doCompute(input, setpoint, timer.timeDiff);
    }
  }
};

// number = bitmap to select for that signature (1 byte, 128-place specifies color codes)
enum PixySignature
{
  PUCK = 1, // orange, 
  GOAL1 = 2, // red
  GOAL2 = 4, // pink
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

PID puckTracker(2, 0, 0, -400, 400);

ScreenPos screenPosPuck;
ScreenPos screenPosGoal1;
ScreenPos screenPosGoal2;

float speedAdjust = 0;

void setup()
{
  Serial.begin(115200); // console output
  Serial1.begin(115200); // communicating w/ ZigBee (for match data)

  delay(500);

  /*
  pixy.init();
  motors.enableDrivers();
  motors.flipM2(true); // flipped so both go forward with positive speeds

  puckTracker.start();
  */
}

void loop()
{
  // fetch latest match data from ZigBee
  match.update();
  
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