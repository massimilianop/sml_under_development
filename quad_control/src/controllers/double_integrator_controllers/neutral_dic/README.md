# DIC = Double Integrator Controller

Double integrator control law, i.e., a function **u**(**p**,**v**) of the position **p** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<img src="https://latex.codecogs.com/svg.latex?\begin{align}&space;&\dot{\mathbf{p}}(t)&space;=&space;\mathbf{v}(t)&space;\notag&space;\\&space;&\dot{\mathbf{v}}(t)&space;=&space;\mathbf{u}(\mathbf{p}(t),\mathbf{v}(t))&space;\notag&space;\end{align}">

NeutralDIC: This DIC does nothing (thus its name)

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}(\mathbf{p},\mathbf{v})&space;=&space;\boldsymbol{0}">
