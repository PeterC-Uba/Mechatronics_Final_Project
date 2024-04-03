struct MatchData
{
  bool status = 0; // true = match in progress
  int time = 0; // seconds
  int x = 0; // cm
  int y = 0; // cm
};
MatchData match;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  
  delay(500);

  printMatchData(match);
  match = parseData(match, "0,0100,---,---");
  printMatchData(match);
  match = parseData(match, "1,0200,050,025");
  printMatchData(match);
  match = parseData(match, "?,???,---,---");
  printMatchData(match);
}

void loop()
{
  // send data from serial => xbee
  if (Serial.available())
  {
    char outgoing = Serial.read();
    Serial1.print(outgoing);
  }

  // receive data from xbee => serial
  String incoming = receiveData();
  if (!incoming.equals("")) // if data actually received
  {
    Serial.print("received data: ");
    Serial.println(incoming);
    match = parseData(match, incoming);
    printMatchData(match);
  }

  delay(100);
}

String receiveData()
{
  String RX = ""; // format: M,TTTT,XXX,YYY

  int length = 0;
  bool isFinished = true;

  if (Serial1.available())
  {
    isFinished = false;
  }

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

MatchData parseData(MatchData previousData, String newData)
{
  String str_status = newData.substring(0, 1);
  String str_time = newData.substring(2, 6);
  String str_x = newData.substring(7, 10);
  String str_y = newData.substring(11, 14);

  // if the new data isn't complete, fall back on previous data
  if (str_status.equals("?") || str_status.equals("/")
  || str_time.equals("????") || str_time.equals("////")
  || str_x.equals("---") || str_y.equals("---"))
  {
    return previousData;
  }

  // new data is complete, go ahead with parsing

  bool isValid = true;
  MatchData updatedData;

  updatedData.status = str_status.equals("1");
  updatedData.time = str_time.toInt();
  updatedData.x = str_x.toInt();
  updatedData.y = str_y.toInt();

  if (updatedData.time < 0 || updatedData.x < 0 || updatedData.y < 0)
  {
    isValid = false;
  }

  // if the new data isn't valid for some reason, fall back on previous data
  if (isValid)
  {
    return updatedData;
  }

  return previousData;
}

void printMatchData(MatchData data)
{
  Serial.println("-----");

  Serial.print("Match in progress: ");
  Serial.println(data.status ? "YES" : "NO");

  Serial.print("Match time: ");
  Serial.print(data.time);
  Serial.println(" s");

  Serial.print("Robot position: (");
  Serial.print(data.x);
  Serial.print(", ");
  Serial.print(data.y);
  Serial.println(")");

  Serial.println("-----");
}
