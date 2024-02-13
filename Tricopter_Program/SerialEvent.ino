void serialEvent() 
{
  while (Serial.available())
  { 
    inChar = Serial.read();
    if (inChar == 'w'){ motor1 += 100;}
    if (inChar == 'q'){ motor1 -= 100;}
    if (inChar == 'e'){ motor1 -= 1000;}
    if (inChar == 's'){ motor2 += 100;}
    if (inChar == 'a'){ motor2 -= 100;}
    if (inChar == 'd'){ motor2 -= 1000;}
    if (inChar == 'x'){ motor3 += 100;}
    if (inChar == 'z'){ motor3 -= 100;}
    if (inChar == 'c'){ motor3 -= 1000;}
   }
}
