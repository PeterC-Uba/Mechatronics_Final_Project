#define BUTTON 4
#define RELAY 3

int buttonState = LOW;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(RELAY, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);


}

void loop() {
  buttonState = digitalRead(BUTTON);
  Serial.println(digitalRead(BUTTON));
  //Serial.print("button pressed");
  if(buttonState == HIGH ){
    digitalWrite(RELAY, HIGH);
    Serial.println("Button is pressed");

  }
  else{
    digitalWrite(RELAY, LOW);
    Serial.println("Button is not pressed");

    //Serial.print("button pressed");
  }
  // put your main code here, to run repeatedly:
  delay(100);

}
