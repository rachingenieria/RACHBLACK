
#include "Arduino.h"
#include "rachblack.h"

extern rachblack vel;

rachblack::rachblack (void)
{
  
}//constructor


void  rachblack::setupconfig(int vavgx,int kpgx, int  kdgx, int pmw_tx,int firware_ver)
{   
    ver = firware_ver;
    vavg = vavgx;
    kpg = kpgx;
    kdg = kdgx;
    pmw_t = pmw_tx;   //VEL VARIA DE 0 a 9
    vel.start = 0xFF;
}
