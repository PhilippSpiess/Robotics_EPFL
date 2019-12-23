#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include <stdio.h>

#define FFT_SIZE 	1024

//extern binary_semaphore_t sendToMatlab_sem;

//2 time FFT_SIZE because these arrays contain complex numbers (real + imaginary)
extern float micLeft_cmplx_input[2 * FFT_SIZE];
extern float micRight_cmplx_input[2 * FFT_SIZE];
extern float micFront_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
extern float micLeft_output[FFT_SIZE];
extern float micRight_output[FFT_SIZE];
extern float micFront_output[FFT_SIZE];

void processAudioData(int16_t *data, uint16_t num_samples);
void doFFT_optimized(uint16_t size, float* complex_buffer);

void calculate_distance(void);
void calculate_angle(float* mic_left,float* mic_right,float* mic_front);

float get_distance(void);
float get_angle(void);
int get_intensite(void);


#endif /* AUDIO_PROCESSING_H */
