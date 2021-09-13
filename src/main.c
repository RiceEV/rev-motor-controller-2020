/****************************************************************************
*
*     Main.c file 
*
******************************************************************************/
#include "bldc_common.h"


// Github Test Comment 1
int main(void)
{
      // DO NOT REMOVE: Manually inserts deadtime before execution of all code
      // to allow debugger time to reflash a possibly bricked device
      for(volatile int i = 0; i<1000000; i++);
      
	__disable_irq();
        //Initializations
        
	peripheral_init();
	device_select_init();
	cafe_init();
	ssp_init(SSPC, SSP_MS_SLAVE);
	__enable_irq();

	
        
       
        PAC55XX_GPIOC->MODE.P4 = IO_PUSH_PULL_OUTPUT;
        configure_timer_b_compare_mode();
       
        /*
        
         uint32_t temp_duty = (uint32_t)(PAC55XX_TIMER_SEL->PRD.w >> 3);
         
         //uint32_t temp_duty = 0;
        
        PAC55XX_TIMER_SEL->CCTR4.CTR = temp_duty;
        PAC55XX_TIMER_SEL->CCTR5.CTR = temp_duty;
        PAC55XX_TIMER_SEL->CCTR6.CTR = temp_duty; 
        */
        
       
      
        motor_dir = 0;

        
        // Braking Set 
        
        braking = 0;

        // Disable Set
        disable_motor = 0;
      
        
        
        //motor_dir = 1;
     
    
        
    
         
	while(1)
		{
                
                
                  
		//while (!millisecond);
		//millisecond = 0;
                
		
                //ssp_write_one(SSPC, (uint32_t)(0xAA) );
                
                set_motor_params(SSPC);

                /*if (motor_
                 if (SPI.DATA_RX.SPEED == 0 && disable motor == 0) {
                    motor_status = motor_active;
                }  */
                /*
		if (motor_status == motor_disabled && SPI.DATA_RX.SPEED)
                      {
                        current_speed = 1;
                        motor_status = motor_active;
                        commutate(firstcomm);
                        
                      }*/
               
       
      
        
	}
}
