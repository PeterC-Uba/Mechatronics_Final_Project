#include <Pixy2.h>

Pixy2 pixy;

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

ScreenPos screenPosPuck;
ScreenPos screenPosGoal1;
ScreenPos screenPosGoal2;

int errorPuck;
int errorGoal1;
int errorGoal2;

void setup()
{
  Serial.begin(115200);

  pixy.init();
}

void loop()
{
  screenPosPuck = pixyScan(PixySignature::PUCK, 3);
  Serial.print("Puck Screen Pos: (");
  Serial.print(screenPosPuck.x);
  Serial.print(", ");
  Serial.print(screenPosPuck.y);
  Serial.print(") => error: ");
  if (screenPosPuck == SCREEN_POS_NULL)
  {
    errorPuck = 0;
    Serial.println("not on screen");
  }
  else
  {
    errorPuck = SCREEN_POS_CENTER.x - screenPosPuck.x;
    Serial.println(errorPuck);
  }

  screenPosGoal1 = pixyScan(PixySignature::GOAL1, 1);
  Serial.print("Goal 1 Screen Pos: (");
  Serial.print(screenPosGoal1.x);
  Serial.print(", ");
  Serial.print(screenPosGoal1.y);
  Serial.print(") => error: ");
  if (screenPosGoal1 == SCREEN_POS_NULL)
  {
    errorGoal1 = 0;
    Serial.println("not on screen");
  }
  else
  {
    errorGoal1 = SCREEN_POS_CENTER.x - screenPosGoal1.x;
    Serial.println(errorGoal1);
  }

  screenPosGoal2 = pixyScan(PixySignature::GOAL2, 1);
  Serial.print("Goal 1 Screen Pos: (");
  Serial.print(screenPosGoal2.x);
  Serial.print(", ");
  Serial.print(screenPosGoal2.y);
  Serial.print(") => error: ");
  if (screenPosGoal2 == SCREEN_POS_NULL)
  {
    errorGoal2 = 0;
    Serial.println("not on screen");
  }
  else
  {
    errorGoal2 = SCREEN_POS_CENTER.x - screenPosGoal2.x;
    Serial.println(errorGoal2);
  }
  Serial.println("");

  delay(100);
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