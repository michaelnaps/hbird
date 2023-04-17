### **Hummingbird Model**

This repository serves as the testing grounds for my joint project in the Boston University classes EC 710/ME 762. I will be attempting to simulate and control a simple hummingbird robot - modelled as a 3-D steered vehicle.

The control paramters are a single lift force pushing the the bird's personal $z$-axes, and two rotations angles; $\theta$ which defines the birds rotation around the world frame $z$-axis, and $\delta$ which defines the bird's offset from the $z$-axis.

The state space form, represented by a second-order system of five continuous parameters is shown below.

$$
\begin{aligned}
     \begin{bmatrix}
        x_1 \\
        x_2 \\
        x_3 \\
        x_4 \\
        x_5
    \end{bmatrix} = \begin{bmatrix}
        x \\
        y \\
        z \\
        \delta \\
        \theta
    \end{bmatrix},
    &&
    \begin{bmatrix}
        x_6 \\
        x_7 \\
        x_8 \\
        x_9 \\
        x_{10}
    \end{bmatrix} = \begin{bmatrix}
        \dot x \\
        \dot y \\
        \dot z \\
        \dot \delta \\
        \dot \theta
    \end{bmatrix}
\end{aligned}
$$

Before describing the equation of motions we can characterize the inputs in terms of the lift force on the COM, and the torque about the two principle axes.

$$
    u = \begin{bmatrix}
        u_1 \\
        u_2 \\
        u_3
    \end{bmatrix} = \begin{bmatrix}
        F \\
        \tau_z \\
        \tau_{xy}
    \end{bmatrix}
$$

Making the dynamics for the hummingbird...

$$
    \dot x = \begin{bmatrix}
        \dot x_1 \\
        \dot x_2 \\
        \dot x_3 \\
        \dot x_4 \\
        \dot x_5 \\
        \dot x_6 \\
        \dot x_7 \\
        \dot x_8 \\
        \dot x_9 \\
        \dot x_{10}
    \end{bmatrix} = \begin{bmatrix}
        \dot x \\
        \dot y \\
        \dot z \\
        \dot \delta \\
        \dot \theta \\
        F \cos(\delta) \cos(\theta) \\
        F \cos(\delta) \sin(\theta) \\
        F \sin(\delta) \\
        \tau_z \\
        \tau_{xy}
    \end{bmatrix} = \begin{bmatrix}
        x_6 \\
        x_7 \\
        x_8 \\
        x_9 \\
        x_{10} \\
        \frac{1}{m} u_1 \cos(x_4) \cos(x_5) \\
        \frac{1}{m} u_1 \cos(x_4) \sin(x_5) \\
        \frac{1}{m} (u_1 \sin(x_4) - mg) \\
        u_2 \\
        u_3
    \end{bmatrix}
$$

In order to collapse this system into the first-order domain, we will implement the discrete system form by exploting the integration function below.

$$
    x_{k+1} = x_k + \int_{t_0}^{t_0+T} f(t,x,u) dt
$$

Because the hummingbird system is time-invariant, we can set $t_0=0$ and solve for $x_{k+1}$. That is,

$$
    x_{k+1} = x_k + Tf(t,x,u)
$$

Making our discrete first-order system dynamics...

$$
    x_{k+1} = \begin{bmatrix}
        x_1 \\
        x_2 \\
        x_3 \\
        x_4 \\
        x_5
    \end{bmatrix} + T \begin{bmatrix}
        \frac{1}{m} (u_1 \cos(x_4) \cos(x_5)) \\
        \frac{1}{m} (u_1 \cos(x_4) \sin(x_5)) \\
        \frac{1}{m} (u_1 \sin(x_4) - mg) \\
        u_2 \\
        u_3
    \end{bmatrix}
$$