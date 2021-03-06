#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <usbcfg.h>

#include <main.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <sensors/proximity.h>
#include <pid_regulator.h>

//Code needed for the use of the Infra-red sensors
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();

    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);

    //starts the PID regulation thread to change the speed variable of the motors every 10ms
    pid_regulator_start();

    //Code needed for the use of the Infra-red sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
    calibrate_ir();
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
