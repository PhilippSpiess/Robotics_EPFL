#include "ch.h"
#include "hal.h"
#include <main.h>

#include <motor.h>
#include <motors.h>
#include <audio_processing.h>
#include <sensors/proximity.h>

#define IR1 0
#define IR2 1
#define IR3 2
#define IR6 5
#define IR7 6
#define IR8 7
#define INTENSITE_SOUND_TRESHOLD 40000
#define IR_TRESHOLD 20
#define COEFF_VIRAGE 2.5

/*
 *function which get the intensity of the sound, the intensite of 6 infrared sensors and the speed determined by the pid
 *to determine what speed to give to the motors in normal case or in case of turning around an obstacle.
*/
void change_speed(int16_t speed_left, int16_t speed_right)
{
		uint32_t intensite_sound = get_intensite();
		uint16_t intensite_infrared_1 = get_calibrated_prox(IR1);
		uint16_t intensite_infrared_8 = get_calibrated_prox(IR8);
		uint16_t intensite_infrared_2 = get_calibrated_prox(IR2);
		uint16_t intensite_infrared_7 = get_calibrated_prox(IR7);
		uint16_t intensite_infrared_3 = get_calibrated_prox(IR3);
		uint16_t intensite_infrared_6 = get_calibrated_prox(IR6);

		if (intensite_sound > INTENSITE_SOUND_TRESHOLD)
		{
			if (intensite_infrared_1 > IR_TRESHOLD || (intensite_infrared_2 > IR_TRESHOLD))  //turn left
			{
				left_motor_set_speed(-speed_right);
				right_motor_set_speed(speed_right);
			}
			else if (intensite_infrared_8 > IR_TRESHOLD || (intensite_infrared_7 > IR_TRESHOLD))  //turn right
			{
				left_motor_set_speed(speed_left);
				right_motor_set_speed(-speed_left);
			}
			else if (intensite_infrared_3 > IR_TRESHOLD ) //turn around right
			{
				if (speed_left > speed_right)
				{
					left_motor_set_speed(speed_left);
					right_motor_set_speed(speed_left/COEFF_VIRAGE);
				}
				else
				{
					left_motor_set_speed(speed_left);
					right_motor_set_speed(speed_right);
				}
			}
			else if (intensite_infrared_6 > IR_TRESHOLD ) //turn around left
			{
				if (speed_left < speed_right)
				{
					left_motor_set_speed(speed_right/COEFF_VIRAGE);
					right_motor_set_speed(speed_right);
				}
				else
				{
					left_motor_set_speed(speed_left);
					right_motor_set_speed(speed_right);
				}
			}
			else
			{

				left_motor_set_speed(speed_left);
				right_motor_set_speed(speed_right);
			}
		}
		else
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
}



