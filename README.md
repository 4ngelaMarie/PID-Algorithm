# PID Algorithm
PID algoirithms are used to bring unstable systems into equilibrium. It's acronym stands for Proportional, Integral and Derivative. For 
this project, the algorithm was used to put a mobile inverted pendulum into unstable equilibrium. The program was run on an arduino 
microcontroller and determined the frequency of voltage bursts delivered to a stepper motor from the Arduino. <br />
The Arduino was delivered real-time data of the pendulum's position using a potentiometer
at the pendulum's pivot. The algoirthm determined the frequency and direction a stepper motion should rotate in 
response to the position (Proportional), angular velocity (Derivative), accumulated distance traveled (Integral.) <br />
For a more thorough description and visual representation, please refer to the MXP_poster pdf in this repository.
