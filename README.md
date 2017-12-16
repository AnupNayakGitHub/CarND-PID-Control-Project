# PID Controller

This project is to control the vehicle around the track by PID controlling method.

## Updated and new files
### Updatded
* main.cpp
* PID.cpp
* PID.h
### New
* twiddler.cpp
* twiddler.h

## Execution Mode
The executable, `pid`, can run in two modes, one is to run in tuning mode and other is to run using user supplied hyper parameters. It can take either no parameter or 3 parameters or 4 parameters.
* First param - Value of P gain
* Second param - Value of I gain
* Third param - Value of D gain
* Forth param - Value to specify to run in tuning or non tuning mode. Value of 1 is true and value of 0 is false.

### Non-Tuning mode
* `./pid 0.5 0 3.0` : P=>0.5, I=>0.0, D=>3.0 and Non tuning 
* `./pid 0.5 0 3.0 0` : P=>0.5, I=>0.0, D=>3.0 and Non tuning 

### Tuning mode
* `./pid 0.5 0 3.0 1` : P=>0.5, I=>0.0, D=>3.0 and tune 

## Non Tuning procedure
The PID controller initialized the Proportional (P), Integration (I), and Derivative (D) gain with the input parameters and calulates the total error based on the `cte` and `speed`. Speed is used instead of time as the derivative depends on how fast the vehicle is moving.

## Tuning procedure
As described below, tuning is done in 3 tuning steps, however, additional steps can be easily added. The user can supply inital values to to start tuning procedure. I have used tuning parameters as 0.5, 0, and 3.0 as P, I, and D respectively.

* The first step is to control the vehicle on the track. To achieve this goal, the tolerance is set to a high value (say 0.1) and only P and D are twicked automatically using the twiddling procedure taught in the lecture. During this procedure the vehicle goes off track and by resetting the simulator the vehicle is brought back on track. After this step the vehicle runs on the track and no need to reset the simulator.

* In the second tuning step, as in the first step, only the P and D are corrected but upto an lower tolerance level (say 0.01). The gain parmaters from step 1 are used to start the second step.

* In the third tuning step, unlike other two steps, only the I gain is manipulated to get a better control of the vehicle.

Please refer the code for further insight of the tuning procedure.

## Visualization
A short video captured from the screen.

[![Watch the video](http://img.youtube.com/vi/WSXdpy05kEM/0.jpg)](https://youtu.be/WSXdpy05kEM)


Observe that the P, I, and D errors go up and down.

This executable is ran for more than 24 hrs continuously. But was excercised only on my Macbook.

