# Comments on Topics and Services

## General comments

1. When service or topic is **added**, add it to **CMakeList.txt**
2. When service or topic is **changed**, perform **catkin_make**


## Description of Services

1. IrisPlusResetNeutral: service for reseting the neutral value that guarantees that the iris+ stays at a desired altitude (no request data is necessary, i.e., requesting service is enough to know that reseting is required)
```
rosservice call IrisPlusResetNeutral
```

2. IrisPlusSetNeutral: service for setting the neutral value
```
rosservice call IrisPlusSetNeutral '{k_trottle_neutral: 1400}'
```

3. TrajDes_Srv: service for setting up a desired trajectory 

```
rosservice call /Iris1/TrajDes_GUI '{trajectory: "StayAtRest" , offset: [0.0,0.0,0.0], rotation: [0.0, 0.0, 0.0], parameters: [0.0]}'
rosservice call TrajDes_GUI '{trajectory: "StayAtRest" , offset: [0.0,0.0,0.0], rotation: [0.0, 0.0, 0.0], parameters: [0.0]}'
```