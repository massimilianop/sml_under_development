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
