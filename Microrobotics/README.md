# Sound_following_robot
C++ program to compile in the Epuck Robot which allows it to follow a 480Hz sound source.
With a thread of 100Hz, a FFT is calcucalted on 3 microphones to triangulate and determine the distance of the sound source in the plane.
The control of the actuators to reach the sound source in done with a PID controller and proximity sensors are used to avoid obstacles.
