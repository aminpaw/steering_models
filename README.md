# Steering Models
## Description
A simulation of a forward and backward steering bicycle model using turtlesim node and ROS.

## Models
### Kinematic Forward Steering 
$$1. \space\space tan(\delta ) = \frac{L_F+L_R}{R}$$

$$2.\space\space tan(\beta) = \frac{L_R}{R} $$

from 1 and 2:

$$ tan(\beta) = \frac{L_R \cdot tan(\delta)}{L_F+L_R}$$

So:

$$\dot{\psi}=\frac{\upsilon}{R}=\frac{\upsilon}{L_F+L_R}(cos(\beta)\cdot tan(\delta))$$

## Reverse Kinematic Model
$$ 1. \space \space tan(\delta) = \frac{L_F+L_R}{R}$$

$$2. \space\space tan(\beta)=\frac{L_F}{R}$$

from 1 and 2 :

$$tan(\beta)=\frac{tan(\delta)\cdot (L_F+L_R)}{L_F}$$

and :

$$\dot{\psi} = \frac{\upsilon \cdot sin(\beta)}{L_R}$$


## Setup :
Open a terminal in workspace location, then :
```bash
$ catkin_make  
$ source devel/setup.bash
```

## Usage :

```bash
$ roslaunch bicycle_model steering_model.launch
```
then in a new terminal : 
```bash
$ cd "workspace location"
$ source run.bash
$ rosservice call /Inputs "velocity: 0.0
delta: 0.0
time: 0.0
model: 0"
```

where :

`velocity`: the linear velocity of the body

`delta`: the steering angle

`time`: the time of motion

`model`: the model simulated `{0 : forward steering, 1 : backward steering, 2 : both models }`

## Parameters

`/Lengths/Front`: the distance between CG and front wheel

`/Lengths/Rear`: the distance between CG and rear wheel

