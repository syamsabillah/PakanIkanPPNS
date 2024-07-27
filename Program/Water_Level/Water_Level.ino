int liquidLevel1 = 0;
int liquidLevel2 = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
}

void loop()
{
  liquidLevel1 = digitalRead(25);
  liquidLevel2= digitalRead(26);
  
}
