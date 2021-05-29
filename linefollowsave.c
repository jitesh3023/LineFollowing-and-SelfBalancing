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
float kp= 0.6;
float ki= 0;
float kd= 0.002;

//Self Balancing Tuning Parameters not required for line following
//..................................
float pitch_kP=  15.1;//5.85;       
float pitch_kI=  0.075;//95;          
float pitch_kD=  9;

float setpoint = 0;
float forward_offset = 2.51;
float forward_buffer = 3.1;
//...................................
/* Motor value constraints
*/
float opt = 67;
float lower_pwm_constrain = 60;
float higher_pwm_constrain = 80;
float left_pwm = 0, right_pwm = 0;
/*
* Line Following PID Variables
*/
float error=0, prev_error, difference, cumulative_error, correction;
int adc_reading[4];
float sensor_value[4];
float sensor_count;

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
sensor_value[0] = constrain(map(adc_reading[0], 1600, 2800, 0, 1000),0,1000);
sensor_value[1] = constrain(map(adc_reading[1], 1600, 2500, 0, 1000),0,1000);
sensor_value[2] = constrain(map(adc_reading[2], 1600, 2200, 0, 1000),0,1000);
sensor_value[3] = constrain(map(adc_reading[3], 1600, 3300, 0, 1000),0,1000);
}

static void calc_error()
{
    int all_black_flag = 1;
    long int weighted_sum = 0, sum = 0, pos = 0;
    
    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 400)  // when bot will be in white region
            all_black_flag = 0;
        
        if(sensor_value[i] > 10)
        {
            weighted_sum += (long)(sensor_value[i]) * (weights[i]*1000);
            sum += sensor_value[i];
        }
        printf("weighted_sum=%lu\n",weighted_sum );
        printf("sum=%lu\n",sum );
        printf("pos=%lu\n",pos);
        return 0;
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
        void bot_stop(MCPWM_UNIT_0 MCPWM_NUM, MCPWM_TIMER_0 TIMER_NUM);

    }

    error = pos;
}
 
static void calc_correction()
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
read_sensors();
// calculate sensor reading in 0 to 1000
calc_sensor_values();
calc_error();
calc_correction();
bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
printf("Error: %f\t",error );
printf("correction: %f ",correction);
printf("Left Pwm: %f\t",left_pwm );
printf("Right Pwm: %f\n",right_pwm );  
}
}
void app_main()
{
	
	mcpwm_initialize();
	nvs_flash_init();
	initialise_wifi();

	xTaskCreate(&line_follow,"line following",10000,NULL,1,NULL);
	xTaskCreate(&http_server,"server",10000,NULL,2,NULL);
    while(1)
    {
        vTaskDelay(1000 / portTICK_RATE_MS);

    }
}
