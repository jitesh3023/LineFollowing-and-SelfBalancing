/*
Copyright (c) 2018, Society of Robotics and Automation, VJTI
This is an example code for line following :
Go to : ..../components/SRA/include/TUNING.h and change EXAMPLE_WIFI_SSID and
EXAMPLE_WIFI_PASS with
your Wifi name and Password.
*/

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "SRA18.h"
#include "TUNING.h"

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};
int weights[4] = {-3,-1,1,3};
//Line Following Tuning Parameters
float kp= 0.4;
float ki= 0;
float kd= 0.2;

//Self Balancing Tuning Parameters not required for line following
//...............................
float pitch_kP=  15.1;//5.85;       
float pitch_kI=  0.075;//95;          
float pitch_kD=  9;

float setpoint = 0;
float forward_offset = 2.51;
float forward_buffer = 3.1;
//...................................
/* Motor value constraints
*/
float opt = 72;
float lower_pwm_constrain =60;
float higher_pwm_constrain = 80;
float left_pwm = 0, right_pwm = 0;
/*
* Line Following PID Variables
*/
float error=0, prev_error, difference, cumulative_error, correction;
int adc_reading[4];
float sensor_value[4];
float sensor_count;
//int n = 1 ;
int lightvalue1 , lightvalue2 , darkvalue1 , darkvalue2;
	int avglightvalue =0, avgdarkvalue=0;
	int sumvalue=0 ; 
	float thresholdvalue=0;
	int c = 0 ;
	int lastseen =0;
	int j = 0 ;
	


static void read_sensors()
{
for(int i = 0; i < 4; i++)
{
adc_reading[i] = adc1_get_raw(channel[i]); //get the direct reading from sensors}
}
}

static void calc_sensor_values()
{
// now constrain sensor reading between 0 to 1000
sensor_value[0] = constrain(map(adc_reading[0], 1801, 4040, 0, 1000),0,1000);
sensor_value[1] = constrain(map(adc_reading[1], 1800, 4046, 0, 1000),0,1000);
sensor_value[2] = constrain(map(adc_reading[2], 1797, 4054, 0, 1000),0,1000);
sensor_value[3] = constrain(map(adc_reading[3], 1797, 4047, 0, 1000),0,1000);
}

static void calc_error()
{
    int all_black_flag = 1;
    long int weighted_sum = 0, sum = 0, pos = 0;
    
    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 800)  // when bot will be in white region
            all_black_flag = 0;
        
        if(sensor_value[i] > 100)
        {
            weighted_sum += (long)(sensor_value[i]) * (weights[i]*1000);
            sum += sensor_value[i];
        }
    }
    
    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(error > 0)
            pos = 2500;
        else
            pos = -2500;
    }

    error = pos;
   // printf("sum ,weighted_sum %lu %lu\n",sum,weighted_sum );
}




void junction()
{
	//left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
//right_pwm = constrain((opt +  correction), lower_pwm_constrain, higher_pwm_constrain);
	//printf("junction\n");
	//int lastseen =0;

	if(sensor_value[0]<450 && sensor_value[3]<450)
	{printf("1\n");
		darkvalue1 = sensor_value[0];
		darkvalue2 = sensor_value[3];

		avgdarkvalue = (darkvalue1 + darkvalue2)/2 ; 

	}
	if(sensor_value[0]>850 && sensor_value[3]>850)
	{
		printf("2\n");
		lightvalue1 = sensor_value[0];
		lightvalue2 = sensor_value[3];

	    avglightvalue = (lightvalue1 + lightvalue2)/2;
	    
	}
   
	sumvalue = avglightvalue + avgdarkvalue ;
	thresholdvalue = sumvalue/2;

	if(c<4)
	{
		printf("if1\n");
		if (sensor_value[0]>(thresholdvalue + 300) && sensor_value[3]>(thresholdvalue + 300) /* condition */)
		
		{
			printf("if2\n");
		   if (lastseen == 0)
		   	{
		   		printf("if3\n");
		   		c = c + 1 ;
		   		/* code */ lastseen = 1;
		   				printf("count%d\n",c );
		   				printf("lastseen%d\n",lastseen);
		   	}	/* code */
		}
		else
		{
			lastseen = 0 ;
		}
		

	}

}


 void STOP ()
{
	 gpio_set_direction(LED_1,GPIO_MODE_OUTPUT);
	gpio_set_direction(LED_2,GPIO_MODE_OUTPUT);

	gpio_set_level(LED_1,1);	//Set LED1 ON
		gpio_set_level(LED_2,1);	//Set LED2 OFF
	

if(sensor_value[0] < 450 && sensor_value[1]< 450 && sensor_value[2] <450 &&
sensor_value[3] <450 ) 
{
	printf("stop\n");

bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
// vTaskDelay(2000/portTICK_PERIOD_MS);

if(c == 1)
{
	gpio_set_level(LED_1,1);	//Set LED1 ON
		gpio_set_level(LED_2,0);	//Set LED2 OFF

		vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms
		//left_pwm=0;
		//right_pwm=70;
		bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0 , 0 , 65);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		//bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
		//j=c;
		c=4;

		

}
else if (c == 2/* condition */)
{
	gpio_set_level(LED_1,0);	//Set LED1 OFF
		gpio_set_level(LED_2,1);	//Set LED2 ON

		vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms/* code */
		//left_pwm=70;
		//right_pwm=70;
		bot_forward(MCPWM_UNIT_0,MCPWM_TIMER_0 , 65 , 65);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		//bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
		//j=c;
		c=4;
		


}
else if (c == 3/* condition */)
{
	/* code */


	gpio_set_level(LED_1,0);	//Set LED1 ON
		gpio_set_level(LED_2,0);	//Set LED2 ON

		vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms
		//left_pwm=70;
		//right_pwm=0;
		bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0 , 65, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		//bot_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
		//j=c;
		c=4;
	

} 
else if(c==4/* condition */)
{
	/* code */

 gpio_set_level(LED_1,0);	//Set LED1 ON
		gpio_set_level(LED_2,0);	//Set LED2 ON

		vTaskDelay(500 / portTICK_PERIOD_MS);	//Wait for 1000ms
		


 		gpio_set_level(LED_1,1);	//Set LED1 OFF
 		gpio_set_level(LED_2,1);	//Set LED2 OFF

 		vTaskDelay(500 / portTICK_PERIOD_MS);
	}	//Wait for 1000ms
}
}
// if (j == 1)

// {
// gpio_set_level(LED_1,0);	//Set LED1 ON
// 		gpio_set_level(LED_2,1);	//Set LED2 OFF

// 		vTaskDelay(3000 / portTICK_PERIOD_MS);	//Wait for 1000ms
// }
// else if (j == 2/* condition */)
// {
// gpio_set_level(LED_1,1);	//Set LED1 ON
// 		gpio_set_level(LED_2,0);	//Set LED2 OFF

// 		vTaskDelay(3000 / portTICK_PERIOD_MS);	//Wait for 1000ms
// 	/* code */
// }
// else if (j == 3)
// {
// gpio_set_level(LED_1,0);	//Set LED1 ON
// 		gpio_set_level(LED_2,0);	//Set LED2 OFF

// 		vTaskDelay(3000 / portTICK_PERIOD_MS);	//Wait for 1000ms

// }
// while(c==0)
// {
// gpio_set_level(LED_1,0);	//Set LED1 ON
// 		gpio_set_level(LED_2,0);	//Set LED2 ON

// 		vTaskDelay(1000 );	//Wait for 1000ms
		


// 		gpio_set_level(LED_1,1);	//Set LED1 OFF
// 		gpio_set_level(LED_2,1);	//Set LED2 OFF

// 		vTaskDelay(1000 );	//Wait for 1000ms
// }














/*
		Set the The LED Pins : GPIO 0 and GPIO 5 to OUTPUT
	

	*/

 

	//switch (n)
	//{
	//case 1 :
	//while(1){
			//Wait for 1000ms
		


		//gpio_set_level(LED_1,1);	//Set LED1 OFF
		////gpio_set_level(LED_2,1);	//Set LED2 OFF

		//vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms
		//break;
           //     printf("cAse1 loop");

		//}
		//break;
		
		//case 2 :
		//while(1){
	
		//gpio_set_level(LED_1,1);	//Set LED1 ON
		//gpio_set_level(LED_2,0);	//Set LED2 ON

		//vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms
		


		//gpio_set_level(LED_1,1);	//Set LED1 OFF
		//gpio_set_level(LED_2,1);	//Set LED2 OFF

		//vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms
		//break;
		//        printf("cAse2 loop");

	    //}
		//break;
		//case 3 :
	//while(1){
		//gpio_set_level(LED_1,0);	//Set LED1 ON
		//gpio_set_level(LED_2,0);	//Set LED2 ON

		//vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms
		

		//gpio_set_level(LED_1,1);	//Set LED1 OFF
		//gpio_set_level(LED_2,1);	//Set LED2 OFF

		//vTaskDelay(1000 / portTICK_PERIOD_MS);	//Wait for 1000ms
		//break;
		//        printf("cAse3 loop");

	//}
		//break;
// void blink_task()
// {
// 	/*
// 		Set the The LED Pins : GPIO 0 and GPIO 5 to OUTPUT
// 	*/

// 	gpio_set_direction(LED_1,GPIO_MODE_OUTPUT);
// 	gpio_set_direction(LED_2,GPIO_MODE_OUTPUT);	

	
// 	{

// 		gpio_set_level(LED_1,0);	//Set LED1 ON
// 		gpio_set_level(LED_2,0);	//Set LED2 ON

// 		vTaskDelay(1000 / 10);	//Wait for 1000ms
		


// 		gpio_set_level(LED_1,1);	//Set LED1 OFF
// 		gpio_set_level(LED_2,1);	//Set LED2 OFF

// 		vTaskDelay(1000 / 10);	//Wait for 1000ms
// 	}
	
// }
	
	
	
	










 
void calc_correction()
{
error *= 0.01;
difference = error - prev_error;
cumulative_error += error;
correction = kp*error + ki*cumulative_error - kd*difference; //PID formula
prev_error = error;

left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);

}


void http_server(void *arg)
{
    printf("%s\n", "http task");
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
     err = netconn_accept(conn, &newconn);
     if (err == ERR_OK) {
       http_server_netconn_serve(newconn,&setpoint,&pitch_kP,&pitch_kD,&pitch_kI,&kp,&kd,&ki, &forward_offset, &forward_buffer);
       netconn_delete(newconn);
     }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

void line_follow(void *arg)
{

vTaskDelay(100/ portTICK_RATE_MS);
while(1)
{
// get the raw readings
	printf("count%d",c);
read_sensors();
// calculate sensor reading in 0 to 1000
calc_sensor_values();

calc_error();
calc_correction();

printf("Error: %f\t",error );
printf("correction: %f ",correction);
printf("Left Pwm: %f\t",left_pwm );
printf("Right Pwm: %f\n",right_pwm );  
junction();

STOP();

bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
//blink_task();

}
}
void app_main()
{
	
	mcpwm_initialize();
	nvs_flash_init();
	initialise_wifi();
	

	xTaskCreate(&line_follow,"line following",100000,NULL,1,NULL);
	//xTaskCreate(&http_server,"server",10000,NULL,2,NULL);
	//xTaskCreate(&junction,"junction",10000,NULL,2,NULL);
}
