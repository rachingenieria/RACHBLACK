#include <arduino.h>
#include <Preferences.h>

Preferences preferences;

#include "rachblack.h"
extern rachblack vel;


void Eeprom_read(void) {
  preferences.begin("my-app", false);
   vel.ver        = preferences.getInt("velver");
   vel.vavg       = preferences.getInt("velvavg");
   
   vel.kdg        = preferences.getInt("velkdg");
   vel.colorlinea = preferences.getInt("velcolorlinea");
   vel.pmw_t      = preferences.getInt("velpmw_t");

   vel.kpg        = preferences.getInt("velkpg");
  preferences.end();

  Serial.printf("FLASH READ vel.ver %d, vel.vavg %d, vel.kpg %d, vel.kdg %d, vel.colorlinea %d, vel.pmw_t %d\n",vel.ver, vel.vavg, vel.kpg, vel.kdg, vel.colorlinea, vel.pmw_t);
}


void Eeprom_save(void) 
{
  preferences.begin("my-app", false);

  preferences.putInt("velver",vel.ver);
  preferences.putInt("velvavg",vel.vavg);
  
  preferences.putInt("velkdg",vel.kdg);
  preferences.putInt("velcolorlinea",vel.colorlinea);
  preferences.putInt("velpmw_t",vel.pmw_t);

  preferences.putInt("velkpg",vel.kpg);

  preferences.end();

  Serial.printf("FLASH SAVE vel.ver %d, vel.vavg %d, vel.kpg %d, vel.kdg %d, vel.colorlinea %d, vel.pmw_t %d\n",vel.ver, vel.vavg, vel.kpg, vel.kdg, vel.colorlinea, vel.pmw_t);
}
