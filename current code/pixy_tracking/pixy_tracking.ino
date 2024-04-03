#include <Pixy2.h>

Pixy2 pixy;

enum PixySignatures
{
  PUCK = 1, // orange
  GOAL1 = 2, // pink
  GOAL2 = 3, // red
};

void setup()
{
  Serial.begin(115200);

  pixy.init();
}

void loop()
{
  pixyScan();
  delay(100);
}

void pixyScan()
{
  /*
  false -> don't wait for block data to continue
  2 -> number of signatures
  */
  pixy.ccc.getBlocks(false, 2, 255);
}