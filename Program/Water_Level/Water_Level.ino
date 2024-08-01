/*!
 * @file  SEN0204.ino
 * @brief  This example is to get liquid level. (Liquid Level Sensor-XKC-Y25-T12V)
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  jackli(Jack.li@dfrobot.com)
 * @version  V1.0
 * @date  2016-1-30
 */

int liquidLevel1 = 0;
int liquidLevel2 = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(24, INPUT);
  pinMode(23, INPUT);
}

void loop()
{
  liquidLevel1 = digitalRead(24);
  liquidLevel2 = digitalRead(23);
  Serial.print("liquidLevel 1= "); Serial.println(liquidLevel1, DEC);
  Serial.print("liquidLevel 2= "); Serial.println(liquidLevel2, DEC);
  delay(500);
}
