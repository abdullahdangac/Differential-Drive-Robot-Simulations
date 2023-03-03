# Differential Drive Robot

A differential wheeled robot is a mobile robot whose movement is based on two separately driven wheels placed on either side of the robot body. It can thus change its direction by varying the relative rate of rotation of its wheels and hence does not require an additional steering motion.

Many mobile robots use a drive mechanism known as differential drive. It consists of 2 drive wheels mounted on a common axis, and each wheel can independently being driven either forward or backward.

<p align="center">
  <img src="https://github.com/abdullahdangac/Differential-Drive-Robot-Simulations/blob/main/images/differential_drive_robot.png" alt="Differential Drive Robot" width="30%" height="30%" title="Differential Drive Robot">
</p>


<br />


## Differential Drive Behavior

<p align="center">
  <img src="https://github.com/abdullahdangac/Differential-Drive-Robot-Simulations/blob/main/images/robot_by_icc.png" alt="Robot by ICC" width="33%" height="33%" title="Robot by ICC">
</p>

$v_r : linear$&nbsp; $velocity$&nbsp; $of$&nbsp; $right$&nbsp; $wheel$  
$v_l : linear$&nbsp; $velocity$&nbsp; $of$&nbsp; $left$&nbsp; $wheel$  
$v : linear$&nbsp; $velocity$&nbsp; $of$&nbsp; $robot$;  
$w : angular$&nbsp; $velocity$&nbsp; $of$&nbsp; $robot$  
$r : radius$&nbsp; $of$&nbsp; $curvature$;  
$l : width$&nbsp; $of$&nbsp; $robot$;

$$v_l = w(r - \frac{l}{2})$$  

$$v_r = w(r + \frac{l}{2})$$   

$$v = \frac{(v_r + v_l)}{2}$$  

$$w = \frac{(v_r - v_l)}{l}$$

$$r = \frac{l}{2}\frac{(v_r + v_l)}{(v_r - v_l)}$$  


<br />


## ICC (Instantaneous Center of Curvature)

If the robot is moving in a curve, there is a center of that curve at that moment, known as the Instantaneous Center of Curvature (or ICC).

<p align="center">
  <img src="https://github.com/abdullahdangac/Differential-Drive-Robot-Simulations/blob/main/images/icc_pose_update.png" alt="ICC Pose Update" width="38%" height="38%" title="ICC Pose Update">
</p>


<br />

ICC point formula:

<p align="center">
  <img src="https://github.com/abdullahdangac/Differential-Drive-Robot-Simulations/blob/main/images/icc_point.png" alt="ICC Point" width="30%" height="30%" title="ICC Point">


$$x' = x - r\sin⁡\theta$$

$$y' = y - r\cos\theta$$  


<br />
<br />


## Rotation Matrix

In linear algebra, a rotation matrix is a transformation matrix that is used to perform a rotation in Euclidean space.

<p align="center">
  <img src="https://github.com/abdullahdangac/Differential-Drive-Robot-Simulations/blob/main/images/rotation.png" alt="Rotation" width="27%" height="27%" title="Rotation">
</p>

$$
\begin{bmatrix} 
x' \\
y' 
\end{bmatrix} = 
\begin{bmatrix} 
\cos\theta & -\sin⁡\theta \\
\sin⁡\theta & \cos\theta
\end{bmatrix} 
\begin{bmatrix} 
x \\
y 
\end{bmatrix}
$$


<br />

rotation matrix with ICC:

$$
\begin{bmatrix} 
x' \\
y' \\
\theta'
\end{bmatrix} = 
\begin{bmatrix} 
\cos w\delta t & -\sin⁡ w\delta t & 0\\ 
\sin⁡ w\delta t & \cos w\delta t & 0\\
0 & 0 & 1
\end{bmatrix} 
\begin{bmatrix} 
x - ICC_x \\
y - ICC_y \\
0
\end{bmatrix} +
\begin{bmatrix} 
ICC_x \\
ICC_y \\
0
\end{bmatrix}
$$


<br />

from here, pose update equations:

$$x' = \cos ⁡w\delta t (x - ICC_x) - \sin ⁡w\delta t (y - ICC_y) + ICC_x$$

$$y' = \sin⁡ w\delta t (x - ICC_x) + \cos⁡ w\delta t (y - ICC_y) + ICC_y$$  

$$\theta' = \theta + w\delta t$$  


<br />
<br />


## Unicycle Model Robot

Unicycle robot, an idealised one-wheeled robot moving in a two-dimensional world, used as an example in control theory problems.

<p align="center">
  <img src="https://github.com/abdullahdangac/Differential-Drive-Robot-Simulations/blob/main/images/unicycle_model.png" alt="Unicycle Model" width="32%" height="32%" title="Unicycle Model">
</p>

<br />
states:

$$
x = \begin{bmatrix} 
x \\
y \\
\phi
\end{bmatrix}
$$


<br />

axis velocities of unicycle model:

$$v_x = \dot x = v\cos⁡\phi$$

$$v_y = \dot y = v\sin⁡\phi$$

$$w = \dot\phi = w$$


<br />

linear velocity $(v)$ and angular veloctiy $(w)$ for differential drive:

$$v = \frac{(v_r + v_l)}{2}$$

$$w = \frac{(v_r - v_l)}{l}$$


<br />

in that case, pose update equations for differential drive:

$$\dot x = \frac{1}{2} (v_r + v_l) \cos⁡\phi$$

$$\dot y = \frac{1}{2} (v_r + v_l) \sin⁡\phi$$  

$$\dot\phi = \frac{1}{l} (v_r - v_l)$$  


<br />
<br />


## Euler's Method

The Euler method is a first-order method, which means that the local error (error per step) is proportional to the square of the step size, and the global error (error at a given time) is proportional to the step size.

<br />

$$\frac{dx}{dt} = f(x,t)$$

$$\frac{\Delta x}{\Delta t} = f(x,t)$$

$\Delta x = x_{n+1} - x_n$

$$\frac{x_{n+1} - x_n}{\Delta t} = f(x_n,t_n)$$

$\Delta t = t_{n+1} - t_n$ &nbsp; &nbsp; $(uniorm$&nbsp; $step$&nbsp; $size)$

$$x_{n+1} = x_n + \Delta t f(x_n, t_n)$$


<br />
<br />

from here, discrete-time pose update equations for differential drive:

$$x' = x + \frac{1}{2} (v_r + v_l) \Delta t \cos⁡\phi$$

$$y' = y + \frac{1}{2} (v_r + v_l) \Delta t \sin⁡\phi$$  

$$\phi' = \phi + \frac{1}{l} (v_r + v_l) \Delta t$$