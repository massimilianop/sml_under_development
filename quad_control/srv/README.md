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

4. SrvControllerChangeByStr: service for ...
```
rosservice call SrvControllerChangeByStr '{controller_name: "a", parameters: "b"}'
```

5. SrvCreateJsonableObjectByStr: service for creating jsonable object
```
rosservice call ServiceChangeMission '{jsonable_name: "IrisSimulatorTrajectoryTracking", string_parameters: "{
"trajectory"	: ["DescribeCircle", 
{
"radius"	: 1.0, 
"rotation"	: [0.0, 0.0, 0.0], 
"speed"	: 0.1, 
"offset"	: [0.0, 0.0, 1.0]
}], 
"controller"	: ["PIDSimpleBoundedIntegralController", 
{
"quad_mass"	: 1.6677, 
"bound_integral_xy"	: 0.0, 
"derivative_gain_z"	: 1.0, 
"derivative_gain_xy"	: 1.0, 
"proportional_gain_z"	: 1.0, 
"integral_gain_z"	: 0.5, 
"integral_gain_xy"	: 0.0, 
"proportional_gain_xy"	: 1.0, 
"bound_integral_z"	: 0.0
}], 
"yaw_controller"	: ["TrackingYawController", 
{
"gain"	: 1.0
}]
}"}'
```

```
rosservice call ServiceChangeMission '{jsonable_name: 'IrisSimulatorTrajectoryTracking', string_parameters: '{"trajectory"	: ["DescribeCircle", {"radius"	: 1.0, "rotation"	: [0.0, 0.0, 0.0], "speed"	: 0.1, "offset"	: [0.0, 0.0, 1.0]}], "controller"	: ["PIDSimpleBoundedIntegralController", {"quad_mass"	: 1.6677, "bound_integral_xy"	: 0.0, "derivative_gain_z"	: 1.0, "derivative_gain_xy"	: 1.0, "proportional_gain_z"	: 1.0, "integral_gain_z"	: 0.5, "integral_gain_xy"	: 0.0, "proportional_gain_xy"	: 1.0, "bound_integral_z"	: 0.0}], "yaw_controller"	: ["TrackingYawController", {"gain"	: 1.0}]}'}'
```

6. PlotService: service for plotting data
```
rosservice call PlotService '{file_path: "/home/pedrootao/SML_CODE/src/quad_control/experimental_data/data/_1461165231_temporary_file1461165231.93.txt"}'
```
