# Sound_following_robot
C++ program to compile in the Epuck Robot which allow it to follow a 480Hz sound source.
A FFT is calcucalte on 3 microphones to determine the intensity on a thread of 100Hz.
A linear regression maps the intensity to the distance of the robot to allow it to adjust the spead with a PID controller.
