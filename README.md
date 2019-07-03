# PID Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---

[//]: # (Image References)

[image1]: ./mov/compiler.png "Complie"
[image2]: ./mov/run.png "run"
[image3]: ./mov/PID_Controller.png "PID"



## Project Introduction

A  proportional–integral–derivative controller [PID](https://en.wikipedia.org/wiki/PID_controller) is a control loop feedback mechanism used to control the car in Udacity's [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45). The simulator provide cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle using [uWebSockets](https://github.com/uWebSockets/uWebSockets). 

## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* For Windows machine [Linux Bash Shell](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)
* Udacity simulator
* Udacity [seed project]https://github.com/udacity/CarND-PID-Control-Project)

---

## Rubric Points

The [rubric](https://review.udacity.com/#!/rubrics/1972/view) points were individually addressed in the [implementation](https://github.com/velsarav/pid-controller)

### Compilation and connection to the simulator
The code compiles correctly.

![alt text][image1]

The PID controller is running and listening on port 4567.

![alt text][image2]

Start the simulator and select project 4 for PID controller

![alt text][image3]

---

## PID Implementation

### Cross Track Error (CTE)
A cross track error is a distance of the vehicle from trajectory. In theory it's best suited to control the car by steering in proportion to CTE.

### P controller
It sets the steering angle in proportion to CTE. It causes the car to steer proportional (opposite) to the car's distance from the lane center (CTE) - if the car is far to the right it steers to the left. By setting the value to 0 the behavior of the car can be found in the [video](./mov/pValue_0.mp4).

```
pid.Init(0, 0.00031, 1.29);
```

### I Controller

It's the integral or sum of error to deal with systematic biases. The I component particularly serves to reduce the CTE around curves.  The "integral" component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. By setting the value to 0 the behavior of the car can be found in the[video](./mov/IValue_0.mp4)

```
pid.Init(0.06, 0.00031, 0);
```

### D Controller

Helps to take temporal derivative of error. This means when the car turned enough to reduce the error, it will help not to overshoot through the x axis. The "differential", component counteracts the P component’s tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing. By setting the value to 0 the behavior of the car can be found in the [video](./mov/DValue_0.mp4)

```
pid.Init(0.06, 0, 1.29);
```

### P, I, D components
The intial value for Kp, Ki, Kd selected using trail and error method.  First, make sure the car can drive straight with zero as parameters. Then add the proportional and the car start going on following the road but it starts overshooting go out of it. Then add the differential to try to overcome the overshooting. The integral part only moved the car out of the road; so we kept it around zero. The twiddle variable set to true, simulator runs the car with confidents till the maximum steps set initially and go through the twiddle algorithm.

```
pid.Init(0.06, 0.00031, 1.29);
```

### Final result
Based on the final parameter the car drives properly and the same captured in the [vidoe](./mov/first_video.mp4) 

---

## Reference
* [Twiddle Algorithm](https://github.com/dkarunakaran/carnd-pid-control-term2-p3)
* [PID simulation](https://github.com/darienmt/CarND-PID-Control-P4)