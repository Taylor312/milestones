#include <Arduino.h>

#if defined(MILESTONE_CAN)
  #include "can/app.cpp"       
#elif defined(MILESTONE_I2C_DAC)
  #include "i2c_dac/app.cpp"  
#elif defined(MILESTONE_LOADCELL)
  #include "loadcell/app.cpp" 
#elif defined(MILESTONE_ENCODER)
  #include "encoder/app.cpp"  
#elif defined(MILESTONE_RS422)
  #include "rs422/app.cpp"    
#elif defined(MILESTONE_VESC_CAN)
  #include "vesc_can/app.cpp" 
#else
  void setup() {}
  void loop() {}
#endif
