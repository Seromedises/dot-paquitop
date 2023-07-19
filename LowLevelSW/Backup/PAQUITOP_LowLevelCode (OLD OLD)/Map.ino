float mapF(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map_51(float IN,float INmax,float OUTmax)
{
  volatile float INlim = INmax/4;
  volatile float b1 = (0.7*OUTmax)/(INmax-INlim);

  volatile float a3 = (2*(5*OUTmax - 5*b1*INmax + 3*b1*INlim))/pow(INlim,3);
  volatile float a4 = -(15*OUTmax - 15*b1*INmax + 8*b1*INlim)/pow(INlim,4);
  volatile float a5 = (3*(2*OUTmax - 2*b1*INmax + b1*INlim))/pow(INlim,5);
  volatile float b0 = OUTmax - b1*INmax;

  if (abs(IN)>=0 && abs(IN)<=INlim){
    if (IN >= 0)
    {
        return a5*pow(IN,5) + a4*pow(IN,4) + a3*pow(IN,3);
    } 
    else
    {
        IN = -IN;
        return -(a5*pow(IN,5) + a4*pow(IN,4) + a3*pow(IN,3));
    } 
  }
  if (abs(IN)>INlim && abs(IN)<=INmax)
  {
    if (IN >= 0)
      {
        return b1*IN + b0;
      }  
    else 
      {
        return b1*IN - b0;
      }
  }
      
}
