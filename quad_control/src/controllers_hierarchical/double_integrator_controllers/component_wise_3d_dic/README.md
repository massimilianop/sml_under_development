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
