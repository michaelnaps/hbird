### **Hummingbird Model**

This repository serves as the testing grounds for my project in the Boston University class ME 762. I will be attempting to simulate and control a simple hummingbird robot - modelled as a 3-D point-mass with assorted dynamics.

The control paramters are a single lift force pushing through the bird's center of mass $z$-axes, and two rotations angles; $\theta$ which defines the birds displacement from the world frame $x$-axis, and $\delta$ which defines the bird's displacement from the $z$-axis.

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
        \frac{1}{m} (F \cos(x_4) \cos(x_5) - w x_6) \\
        \frac{1}{m} (F \cos(x_4) \sin(x_5) - w x_7) \\
        \frac{1}{m} (F \sin(x_4) - w x_8 - mg) \\
        \tau_z \\
        \tau_{xy}
    \end{bmatrix} = \begin{bmatrix}
        x_6 \\
        x_7 \\
        x_8 \\
        x_9 \\
        x_{10} \\
        \frac{1}{m} (u_1 \cos(x_4) \cos(x_5) - w x_6) \\
        \frac{1}{m} (u_1 \cos(x_4) \sin(x_5) - w x_7) \\
        \frac{1}{m} (u_1 \sin(x_4) - w x_8 - mg) \\
        u_2 \\
        u_3
    \end{bmatrix}
$$

Where the scalar coefficient $w$ describes the air coefficient of friction.
