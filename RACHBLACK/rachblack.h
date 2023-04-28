#ifndef RACHBLACK_h
#define RACHBLACK_h
#include "Arduino.h"

//--------------------------------------------------------------------------------------//



//ESTRUCTURA PARA SALVAR DATOS
class rachblack
{
  public:
    rachblack (void);
    void setupconfig(int vavg,int kpg, int  kdg, int pmw_t,int firware_ver);

        int ver;
        int vavg;
        int kpg;
        int kdg;
        int pmw_t; 
        
        int colorlinea; 
        int position_line;
        int start;
        int remoto_enable;
        int sw_enable;
        int mx, my;
        
  private:
        
        //variables de control
};
 
#endif