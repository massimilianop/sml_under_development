# DIC = Double Integrator Controller

This package provides double integrator control laws, i.e., a function **u**(**x**,**v**) of the position **x** and velocity **v**, which guarantees asymptotic convergence of the position to **0** along trajectories of the closed loop system

<pre>
d/dt <b>x</b> = <b>v</b>
d/dt <b>v</b> = <b>u</b>(<b>x</b>,<b>v</b>)
</pre>

<img src="https://latex.codecogs.com/svg.latex?\mathbf{u}(\mathbf{p},\mathbf{v})&space;=&space;-&space;k_{\scriptsize{p}}&space;\boldsymbol{\sigma}_{\scriptsize{p}}(\mathbf{p})&space;-&space;k_{\scriptsize{v}}&space;\boldsymbol{\sigma}_{\scriptsize{v}}(\mathbf{v})&space;">
