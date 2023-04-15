### **Hummingbird Model**

This repository serves as the testing grounds for my joint project in the Boston University classes EC 710/ME 762. I will be attempting to simulate and control a simple hummingbird robot - modelled as a 3-D steered vehicle.

The control parameters are a single lift force pushing through the center of mass (COM) and norm to the bird's x-y axis and the appropriate Euler angles (check?).

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
        u_1 \cos(x_4) \cos(x_5) \\
        u_1 \cos(x_4) \sin(x_5) \\
        u_1 \sin(x_4) \\
        u_2 \\
        u_3
    \end{bmatrix}
$$
