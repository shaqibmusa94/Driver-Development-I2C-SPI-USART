/*
 * stm32_test_lcd.c
 *
 *  Created on: 22 Mar 2022
 *      Author: USER
 */

#include "stm32f407xx.h"
#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK		16000000UL

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}


char* get_day_of_week(uint8_t i)
{
	char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday"};
	return days[i - 1];
}

void number_to_string(uint8_t num, char* buf)
{
	if(num < 10)
	{
		buf[0] = '0';
		buf[1] = num + 48;						//ascii code
	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num / 10) + 48;
		buf[1] = (num % 10) + 48;
	}
}

char* time_to_string(RTC_time_t *rtc_time)
{
	static char buf[9];

	buf[2] = ':';
	buf[5] = ':';

	number_to_string(rtc_time->hours, buf);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;
}

char* date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2] = '/';
	buf[5] = '/';

	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	buf[8] = '\0';

	return buf;
}




int main()
{

	printf("RTC Test\n");

	RTC_time_t current_time;
	RTC_date_t current_date;

	lcd_init();
	lcd_print_string("LCD Test..");
	lcd_display_clear();
	lcd_print_string("New word");
	lcd_set_cursor(2,1);
	lcd_print_string("New line");

	if(ds1307_init())									//if return 1, RTC is halt (CH set to 1)
	{
		printf("RTC init has failed\n");
		while(1);
	}

	current_date.day = FRIDAY;
	current_date.date = 10;
	current_date.month = 1;
	current_date.year = 21;

	current_time.seconds = 10;
	current_time.minutes = 3;
	current_time.hours = 2;
	current_time.time_format = TIME_FORMAT_12HRS_PM;

	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	init_systick_timer(1);

	ds1307_get_current_time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format) ? "PM" : "AM";   							//if current time format is equal to 1, the it is PM
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
	}else
	{
		printf("Current time = %s\n", time_to_string(&current_time));
	}

	ds1307_get_current_date(&current_date);
	// 15/01/21 <friday>
	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
	printf("Address of current time in main fx is %p\n", &current_time);
	return 0;
}


void SysTick_Handler()
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	ds1307_get_current_time(&current_time);
	printf("Address of current time in systick handler is %p\n", &current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format) ? "PM" : "AM";   							//if current time format is equal to 1, the it is PM
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
	}else
	{
		printf("Current time = %s\n", time_to_string(&current_time));
	}

	ds1307_get_current_date(&current_date);
	// 15/01/21 <friday>
	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
}

