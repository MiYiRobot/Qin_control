#include "main.h"

void move_task(void *pvParameters)
{
    while(1)
    {        
       if(Mode == remt_ctrl_mode)   //��ģң������123�ص��̣�5����������6����̧���½�
       {
         if(PPM_Databuf[1] != 0 || PPM_Databuf[0] != 0 || PPM_Databuf[3] != 0)
         {   
            Speed_Chassis_Calculate(); 
            upper_control();
         } 
         
       }
       else if(Mode == go_to_p_mode)
       {
         Speed_x = Cmd_x;
         Speed_y = Cmd_y;
         Speed_w = Cmd_w;
       }
       else if(Mode == auto_mode)
       {
       
       }

         vTaskDelay(5);
    }  
}




