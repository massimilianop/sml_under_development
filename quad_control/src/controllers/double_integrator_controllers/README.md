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


# DIC = Double Integrator Controller

Double integrator control law, i.e., a function **u**(**p**,**v**) of the position **p** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<img src="https://latex.codecogs.com/svg.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align}">

Particularly

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}\in\mathcal{C}^{2}(\mathbb{R}^{3}\times\mathbb{R}^{3},\mathbb{R}^{3})">

where

<img src= "https://latex.codecogs.com/svg.latex?\mathbf{u}(\mathbf{p},\mathbf{v})&space;=&space;\begin{bmatrix}&space;u(p_{\scriptsize{x}},v_{\scriptsize{x}})&space;\\&space;u(p_{\scriptsize{y}},v_{\scriptsize{y}})&space;\\&space;u(p_{\scriptsize{z}},v_{\scriptsize{z}})&space;\end{bmatrix}" >

i.e, the 3D controller is in fact, 3 component wise controllers.

Moreover,

<img src="https://latex.codecogs.com/svg.latex?u(p,v)=-k_{\scriptsize{p}}&space;\sigma_{\scriptsize{p}}(p)-k_{\scriptsize{v}}\sigma_{\scriptsize{v}}(v)">

where

<img src="https://latex.codecogs.com/svg.latex?\sigma_{\scriptsize{p}}(p)=&space;\frac{p}{\sqrt{\sigma_{\scriptsize{p}}^2&plus;p^2}},\sigma_{\scriptsize{v}}(v)=&space;\frac{v}{\sqrt{\sigma_{\scriptsize{v}}^2&plus;v^2}}">




# DIC = Double Integrator Controller

Double integrator control law, i.e., a function **u**(**p**,**v**) of the position **p** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<img src="https://latex.codecogs.com/svg.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align}">

Particularly

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}\in\mathcal{C}^{2}(\mathbb{R}^{n}\times\mathbb{R}^{n},\mathbb{R}^{n})">

where n is determined by the controller, by checking the size of the position state **pp**.  The control law is given by 

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}(\mathbf{p},\mathbf{v})&space;=&space;-&space;k_{\scriptsize{p}}&space;\boldsymbol{\sigma}_{\scriptsize{p}}(\mathbf{p})&space;-&space;k_{\scriptsize{v}}&space;\boldsymbol{\sigma}_{\scriptsize{v}}(\mathbf{v})&space;">

where we emphasize that all components of the state affect each component of the control law (i.e., control in x-direction not decoupled from y-direction).

Moreover

<img src="https://latex.codecogs.com/svg.latex?\boldsymbol{\sigma}_{\scriptsize{p}}(p)=\frac{\mathbf{p}}{\sqrt{\sigma_{\scriptsize{p}}^2&plus;\|&space;\mathbf{p}\|^2}},\boldsymbol{\sigma}_{\scriptsize{v}}(v)=\frac{\mathbf{v}}{\sqrt{\sigma_{\scriptsize{v}}^2&plus;\|&space;\mathbf{v}\|^2}}.">


# DIC = Double Integrator Controller

Double integrator control law, i.e., a function **u**(**p**,**v**) of the position **p** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<img src="https://latex.codecogs.com/svg.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align}">

NeutralDIC: This DIC does nothing (thus its name)

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}(\mathbf{p},\mathbf{v})&space;=&space;\boldsymbol{0}">


# DIC = Double Integrator Controller

Double integrator control law, i.e., a function **u**(**p**,**v**) of the position **p** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<img src="https://latex.codecogs.com/svg.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align}">

Particularly

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}\in\mathcal{C}^{2}(\mathbb{R}^{3}\times\mathbb{R}^{3},\mathbb{R}^{3})">

which is a 3D double integrator controller. The control law is given by

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}(\mathbf{p},\mathbf{v})&space;=&space;-&space;k_{\scriptsize{p}}&space;\boldsymbol{\sigma}_{\scriptsize{p}}(\mathbf{p})&space;-&space;k_{\scriptsize{v}}&space;\boldsymbol{\sigma}_{\scriptsize{v}}(\mathbf{v})&space;">

where we emphasize that all components of the state affect each component of the control law (i.e., control in x-direction not decoupled from y-direction).

Moreover

<img src="https://latex.codecogs.com/svg.latex?\boldsymbol{\sigma}_{\scriptsize{p}}(p)=\frac{\mathbf{p}}{\sqrt{\sigma_{\scriptsize{p}}^2&plus;\|&space;\mathbf{p}\|^2}},\boldsymbol{\sigma}_{\scriptsize{v}}(v)=\frac{\mathbf{v}}{\sqrt{\sigma_{\scriptsize{v}}^2&plus;\|&space;\mathbf{v}\|^2}}.">



# DIC = Double Integrator Controller

Double integrator control law, i.e., a function **u**(**p**,**v**) of the position **p** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<img src="https://latex.codecogs.com/svg.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align}">

Particularly

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}\in\mathcal{C}^{2}(\mathbb{R}^{n}\times\mathbb{R}^{n},\mathbb{R}^{n})">

where n is determined by the controller, by checking the size of the position state **pp**.  The control law is given by 

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}(\mathbf{p},\mathbf{v})&space;=&space;-&space;k_{\scriptsize{p}}&space;\boldsymbol{\sigma}_{\scriptsize{p}}(\mathbf{p})&space;-&space;k_{\scriptsize{v}}&space;\boldsymbol{\sigma}_{\scriptsize{v}}(\mathbf{v})&space;">

where we emphasize that all components of the state affect each component of the control law (i.e., control in x-direction not decoupled from y-direction).

Moreover

<img src="https://latex.codecogs.com/svg.latex?\boldsymbol{\sigma}_{\scriptsize{p}}(p)=\frac{\mathbf{p}}{\sqrt{\sigma_{\scriptsize{p}}^2&plus;\|&space;\mathbf{p}\|^2}},\boldsymbol{\sigma}_{\scriptsize{v}}(v)=\frac{\mathbf{v}}{\sqrt{\sigma_{\scriptsize{v}}^2&plus;\|&space;\mathbf{v}\|^2}}.">
