#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#endif

#include "main.h"
#include "board_init.c"
#include "sonar.c"
#include "swarmfunctions.c"
#include "communication.c"

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{
	jiffies++;	// Timers

	// set flags every 100ms
	if(jiffies%100 == 0)
	{
		servo_motor_on   = true;
		sendmessage_fast = true;	
	}

	if(jiffies%200 == 0)
	{
		use_sensor_data_on = true;
		cnt4sensor++;
		
		if(sonar_attached)
		{
			sensor_value = get_sonar_value();
			
			if(sensor_value != 0 && sensor_value < RANGE3)
			{
				sensor_value_now = sensor_value;
				if(dblchk) sum_dbl += sensor_value_now;
				if(trichk) sum_tri += sensor_value_now;
			}
			else {dblchk = false; trichk = false;}

			if(cnt4sensor%2 == 0)
			{ 
				if(dblchk) 	sensor_value_dblchk = sum_dbl / 2;
				else 		sensor_value_dblchk = 0;
				sum_dbl = 0;
				dblchk = true;
			}
			
			if(cnt4sensor%3 == 0)
			{
				if(trichk) 	sensor_value_trichk = sum_tri / 3;
				else 		sensor_value_trichk = 0;
				sum_tri = 0;
				trichk = true;
			}
		}
			
		display_on = true;


//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR
//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR
//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR//INDICATOR

//LED_PORT.OUTTGL = LED_USR_1_PIN_bm;	
//LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	
	}
	
	if(jiffies%1000 == 0)
	{		
		if(communication_on) sec_counter++;
		sync = true;		//synchro bit should be set every 1 sec
		rhythm_on = true;
		sensor_value_now = 0;

		if(!communication_on) LED_PORT.OUT =  LED_USR_0_PIN_bm;
		if(communication_on)  LED_PORT.OUT = !LED_USR_0_PIN_bm;		
	}
	xgrid.process();
}

// heartbeat (left bottom device)
void StageInit(int StageTime, const char str[])
{
	if(sec_counter == StageTime && special)
	{
		send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, str);
		init_variables();
	}
}

// ============================================================================================
// MAIN FUNCTION
// ============================================================================================
int main(void)
{
	float angle = 0;
	// last key pressed
	char last_press = 0;
	
	xgrid.rx_pkt = &rx_pkt;

	_delay_ms(50);

	// ========== INITIALIZATION ==========
    init();				//for board
	init_servo();		//for servo
	// initialize variables in proto.
	// FIXME enables the servos too.
	//init_variables();	//for program
	init_sonar();		//for sensor

	fprintf_P(&usart_stream, PSTR("START (build number : %ld)\r\n"), (unsigned long) &__BUILD_NUMBER);

	// ===== SONAR CHECK & Indicated by LED (attached/not = GREEN/RED) =====
	sonar_attached = check_sonar_attached();	//1:attached, 0:no

	//  visual indicator of sonar check (disabled)
	//if(sonar_attached)	{LED_PORT.OUT = LED_USR_2_PIN_bm; _delay_ms(2000);}
	//else				{LED_PORT.OUT = LED_USR_0_PIN_bm; _delay_ms(2000);}

	// determine connectivity (connected global)
	temp_time = jiffies + 2000;
	while(jiffies < temp_time)
	{
		// send dummy data
		if(sendmessage_fast)
		{
			send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "@");
			sendmessage_fast = false;
		}
	}
	// detect left bottom corner in proto
	// ===== Identification of Left Bottom Corner module =====
	// Special module is necessary
	// 1) as a pace maker in "rhythm" mode,
	// 2) as a messanger of variable-reset signal
	// if(!connected[0] && connected[2] && connected[3] && !connected[4]) special = true;
	

	// #################### MAIN LOOP ####################

	while (1)
	{
		// ========== REBOOT PROCESS ==========
		if(reboot_on)
		{
			temp_time = jiffies + 3000;
			while(jiffies < temp_time){LED_PORT.OUTTGL = LED_USR_1_PIN_bm; _delay_ms(100);}
			xboot_reset();	
		}
			
		// ========== KEY INPUT ==========
		last_press = key_input(); // TODO remove side effects from key input

		// ========== CALCULATION ==========
		// this seems to be initialization
		if(sonar_attached && !communication_on)
		{
			// waits for sensor value to reach threshold RANGE1. Why?
			if(sensor_value_trichk > RANGE1 && sensor_value_trichk < RANGE3)
			{
				send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "R"); // run mode
				_delay_ms(100);
				send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "1"); // first phase
				communication_on = true;
			}
		}

		// main loop. Flag set at initialization
		if(communication_on)
		{
			// ========== SENSOR DATA ==========
			//SENSOR: works every 200 ms (See ISR() function)

			// ========== SELECT BEHAVIOR ==========
			// when sendmessage_fast flag set (e.g. every 100ms)
			if(sendmessage_fast)
			{
				// SCENARIO
				// === STAGE 1: Ken's model
				if(sec_counter < STGtime2) 
					angle = 90 * Sync_Disorder(STGtime2);
//angle = 90 * rhythm_control3();

				// === STAGE 2: Rhythm 2 (Frequency Modulation)
				// these next two lines are sec_counter = max(message from bottom left, sec_counter)
				// the bottom left sends a message to devices to advance their counter, or they may advance on their own
				StageInit(STGtime2, "2");
				if(sec_counter > STGtime2 && sec_counter < STGtime3)
					angle = 45 * rhythm_control2();


				// === STAGE 3: Rhythm 1 (Amplitude Modulation)
				StageInit(STGtime3, "3");
				if(sec_counter > STGtime3 && sec_counter < STGtime4)
					angle = 90 * rhythm_control1();


				// === STAGE 4: Break
				StageInit(STGtime4, "4");
				if(sec_counter > STGtime4 && sec_counter < STGtime5)
					disable_servo(); // What does this do?


				// === STAGE 5: Ken's model
				StageInit(STGtime5, "5");
				if(sec_counter > STGtime5 && sec_counter < STGtime6)
					angle = 75 * Sync_Disorder(STGtime6);


				// === STAGE 6: Column swing
				StageInit(STGtime6, "6");
				if(sec_counter > STGtime6 && sec_counter < STGtime7)
					angle = 90 * column();


				// === STAGE 7: Horizontal propagation
				StageInit(STGtime7, "7");
				if(sec_counter > STGtime7 && sec_counter < STGtime8)
					angle = 90 * plane_wave();


				// === STAGE 8: Avalanche
				StageInit(STGtime8, "8");
				if(sec_counter > STGtime8 && sec_counter < STGtime9)
					angle = 90 * avalanche();


				// === STAGE 8: Break
				StageInit(STGtime9, "9");
				if(sec_counter > STGtime9 && sec_counter < LASTtime)
					disable_servo();


				// === Waiting for next run
				StageInit(LASTtime, "0"); // advance all timers to LASTtime

				// flood reboot message
				if(sec_counter > LASTtime)
				{
					send_message(MESSAGE_COMMAND, ALL_DIRECTION, ALL, "Z");
					temp_time = jiffies + 3000;
					while(jiffies < temp_time);
					xboot_reset();
				}

				sendmessage_fast = false;
			}
		
			// ========== SERVO MOTOR CONTROL ==========
			// when flag set (e.g. every 100ms)
			// note that the servo may be disabled or enabled
			if(servo_motor_on)
			{
				set_servo_position(angle);
				servo_motor_on = false;
			}
		}

		// ==========  DISPLAY ==========
		// debugging information (different than debug mode)
		// display_on: message sent to main
		// display: global debug flag
		if(display_on && display)
		{
			if(!communication_on) fprintf_P(&usart_stream, PSTR("NO COMM.\r\n"));
			else
			{
				if(sonar_attached)
				fprintf_P(&usart_stream, PSTR("sec cnt=%d SONAR=%d, dblchk=%d,trichk=%d\r\n"),sec_counter, sensor_value, sensor_value_dblchk, sensor_value_trichk);
				else
				fprintf_P(&usart_stream, PSTR("sec cnt=%d\r\n"),sec_counter);
			}
			//fprintf_P(&usart_stream, PSTR("hd: %i.%i\n\r"), prt_flt3(agent1.hd));
			display_on = false;
		}
	}
	
	return 0;
}

