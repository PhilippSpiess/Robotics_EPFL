#include "ch.h"
#include "hal.h"
#include <main.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <arm_const_structs.h>


//2 time FFT_SIZE because these arrays contain complex numbers (real + imaginary)
float micLeft_cmplx_input[2 * FFT_SIZE];
float micRight_cmplx_input[2 * FFT_SIZE];
float micFront_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
float micLeft_output[FFT_SIZE];
float micRight_output[FFT_SIZE];
float micFront_output[FFT_SIZE];


#define MIN_VALUE_THRESHOLD	10000
#define MIN_FREQ		31	//we only analyse the frequency equivalent to an index of 31 or 32
#define MAX_FREQ		32
#define DECALAGE_ANGLE 10
#define ANGLE_LIMITE 90
#define INTENSITY_TRESHOLD 50000
#define PENTE_DISTANCE 0.00009
#define CONSTANTE_DISTANCE 29.628
#define DISTANCE_LIMITE 17
#define NB_MICROS 4

static float distance;
static float angle;
static int intensite_sound; // 32 bits because it can go up to millions


//function to calculate the distance to the corresponding intensity of the front microphone
void calculate_distance()
{
	if (intensite_sound>INTENSITY_TRESHOLD)
	{
		 distance= -intensite_sound*PENTE_DISTANCE + CONSTANTE_DISTANCE;
		 if (distance<0) {distance=0;}
	}
	else
	{
		distance = DISTANCE_LIMITE;
	}
}

//function to calculate the angle as a function of the front, the left and the right microphone intensity detected
void calculate_angle(float* mic_left,float* mic_right,float* mic_front)
{

	float max_right = MIN_VALUE_THRESHOLD;
	float max_left = MIN_VALUE_THRESHOLD;
	float max_front = MIN_VALUE_THRESHOLD;


	for(uint8_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if(mic_right[i] > max_right){
				max_right = mic_right[i];
			}
		}

	for(uint8_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(mic_left[i] > max_left){
			max_left = mic_left[i];
		}
	}

	for(uint8_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if(mic_front[i] > max_front){
				max_front = mic_front[i];
			}
		}

	float moyenne = (max_right + max_left + max_front)/3;
	float diff;
	if (max_front < max_right && max_front < max_left)
	{
		if (max_right > max_left){angle = ANGLE_LIMITE;}
		if (max_left > max_right){angle = -ANGLE_LIMITE;}
	}
	if (max_front > max_right || max_front > max_left)
	{
		if (max_right > max_left)
		{
		diff= sqrt(abs(max_right-max_left)/(moyenne)); //TROP LOURD?
		angle = ANGLE_LIMITE * diff ;
		}
		else if (max_left > max_right)
		{
		diff= sqrt(abs(max_right-max_left)/(moyenne));
		angle = -ANGLE_LIMITE * diff ;
		}
	}

	intensite_sound = max_front;
	calculate_distance();
}



/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get for each microphone (should always be 160)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=NB_MICROS){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i];
		micLeft_cmplx_input[nb_samples] = (float)data[i+1];
		micFront_cmplx_input[nb_samples] = (float)data[i+2];
		// MicBack is not needed for our project

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		//...// compute magnitude for each micro
		//...// algo to determine the position of sonor source selon ref polaire


		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*	Used to compute intensity and sound direction
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		calculate_angle(micRight_output,micLeft_output,micFront_output);
		nb_samples = 0;
		}
}

/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of trick to optimized the computations
*/
void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == FFT_SIZE)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

//function to send the distance to pid_regulator.c to compute the speed which is then sent it to motor.c
float get_distance(void){
	return distance;
}

//function to send the angle to pid_regulator.c to compute the speed which is then sent to motor.c
float get_angle(void){
	return angle;
}

//function to send the sound intensity of the front microphone
//to motor.c to determine different speed situations of the right and left motors.
int get_intensite(void){
	return intensite_sound;
}
