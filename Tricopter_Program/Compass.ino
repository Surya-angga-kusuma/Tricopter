void compass_update()
{
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  //non tilt compentation
//  heading = atan2(y, x);
  
  //tilt compentation
  compass_tiltcompentation();
  heading = atan2(Yh, Xh);
  
  declinationAngle = 0.56;
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  headingDegrees = (heading * 180 / M_PI);
}

void compass_tiltcompentation()
{
  normXAxis = x;
  normYAxis = y;
  normZAxis = z;
  
  acc_x = ax*2.0 / 32768.0;
  acc_y = ay*2.0 / 32768.0;

  acc_x = constrain(acc_x, -1, 1);
  acc_y = constrain(acc_y, -1, 1);

  compensatePitch = asin(acc_x);
  compensateRoll  = asin(-acc_y);

  cosComRoll  = cos(compensateRoll);
  sinComRoll  = sin(compensateRoll);
  cosComPitch = cos(compensatePitch);
  sinComPitch = sin(compensatePitch);

  Xh = normZAxis*sinComPitch + normXAxis*cosComPitch;
  Yh = normYAxis*sinComRoll*sinComPitch + normYAxis*cosComRoll - normZAxis*sinComRoll*cosComPitch;
}
