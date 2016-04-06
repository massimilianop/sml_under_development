# DIC = Double Integrator Controller

This package provides double integrator control laws, i.e., a function **u**(**x**,**v**) of the position **x** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<img src="https://latex.codecogs.com/svg.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align}">

The class DoubleIntegratorController() in double_integrator_controller.py provides an abstract class for double integrator controllers. It should be taken as a template for constructing a new DI controller.

When constructing a new controller

1. mkdir **meaningful_name_dic**
2. cp double_integrator_controller.py meaningful_name_dic/meaningful_name_dic.py
3. modify class inside to your own taste: MeaningfulNameDIC()
4. include the new class MeaningfulNameDIC() in database_dic.py
  1. import meaningful_name_dic/meaningful_name_dic
  2. **database_dic["MeaningfulNameDIC"] = meaningful_name_dic.meaningful_name_dic.MeaningfulNameDIC**


![alt text][logo2]

[logo2]: https://github.com/adam-p/markdown-here/raw/master/src/common/images/icon48.png "Logo Title Text 2"

![alt text][logo]

[logo]: https://latex.codecogs.com/png.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align} "Logo Title Text 2"


[![alt text](https://latex.codecogs.com/png.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align})](https://latex.codecogs.com/png.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align})