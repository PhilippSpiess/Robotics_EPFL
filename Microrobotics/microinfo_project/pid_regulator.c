#include "ch.h"
#include "hal.h"
#include <math.h>

#include <main.h>
#include <motor.h>
#include <audio_processing.h>


#define GOAL_DISTANCE 3
#define ERROR_THRESHOLD 0.5
#define MAX_SUM_ERROR 150
#define ROTATION_THRESHOLD 100
#define ROTATION_COEFF 0.8
#define KP 70
#define KI 5
#define KD 0.01 // small but the e-puck can oscillate if we make it too big
#define INTERVAL_TIME 0.01
#define WAIT_MS_THD 10
#define WORKING_AREA_THD 256


//simple PID regulator implementation
int16_t pid_regulator(float distance, float goal){

	float error = 0;
	int16_t speed = 0;

	static float sum_error = 0;
	static float pre_error = 0;

	error = distance - goal;

	//disables the PID regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error + KD* (error-pre_error)/INTERVAL_TIME;
	pre_error = error;
    return (int16_t)speed;
}


static THD_WORKING_AREA(waPidRegulator, WORKING_AREA_THD);
static THD_FUNCTION(PidRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed;
    int16_t speed_right;
    int16_t speed_left;

    while(1){
        time = chVTGetSystemTime();
        float distance = get_distance();
        float angle = get_angle();

        //computes the speed to give to the motors
        	speed = pid_regulator(distance, GOAL_DISTANCE);

        //computes a correction factor to let the robot rotate to the sound source
        float speed_correction = angle*distance;

        //if the sound is nearly in front of the front microphone, the e-puck don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){speed_correction = 0;}

        //applies the speed from the PID regulator and the correction for the rotation

		speed_right = (speed - ROTATION_COEFF * speed_correction);
		speed_left = (speed + ROTATION_COEFF * speed_correction);

		change_speed(speed_left, speed_right);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(WAIT_MS_THD));
    }
}

void pid_regulator_start(void){
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}
