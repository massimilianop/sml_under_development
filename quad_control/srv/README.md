# Comments on Topics and Services

## General comments

1. When service or topic is **changed**, perform **catkin_make**

## Description of Services

1. TrajDes_Srv: service for setting up a desired trajectory 

```
rosservice call /Iris1/TrajDes_GUI '{trajectory: "StayAtRest" , offset: [0.0,0.0,0.0], rotation: [0.0, 0.0, 0.0], parameters: [0.0]}'
```