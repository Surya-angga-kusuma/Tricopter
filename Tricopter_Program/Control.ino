void control_update()
{
  if(ch5==0)
  {
    throttle = 1000;
    servo  = servoAngleInit;
    motor1 = 1000;
    motor2 = 1000;
    motor3 = 1000;
  }

  if(ch5==1)
  {
    motor1=throttle;
  }

}
