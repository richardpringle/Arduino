#include "C:\Program Files (x86)\Arduino\hardware\arduino\sam\libraries\Pwm01\pwm01.h"

   uint32_t  pwm_duty = 65535;
   uint32_t  pwm_freq1 = 2;  
   uint32_t  pwm_freq2 = 21000;

void setup() 
{


   // Set PWM Resolution
   pwm_set_resolution(16);  

   // Setup PWM Once (Up to two unique frequencies allowed
   //-----------------------------------------------------    
   pwm_setup( 8, pwm_freq2, 2);  // Pin 8 freq set to "pwm_freq2" on clock B
   pwm_setup( 9, pwm_freq2, 2);  // Pin 9 freq set to "pwm_freq2" on clock B
     

}

void loop() 
{ 
     // Write PWM Duty Cycle Anytime After PWM Setup
   //-----------------------------------------------------    
   pwm_write_duty( 8, pwm_duty );  // 50% duty cycle on Pin 8
   pwm_write_duty( 9, pwm_duty );  // 50% duty cycle on Pin 9


   delay(1000);  // 30sec Delay; PWM signal will still stream
       
   // Force PWM Stop On All Pins
   //-----------------------------    
   pwm_stop( 8 );
   pwm_stop( 9 );
}
