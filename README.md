# ModelBasedControl_CartPole
Model Based Control System Design for a Cart with Inverted Pole.

The control strategy of ***Control Partioning*** is used where the controller actuator not only depends on the controller gain paramters but also the model parameters. Thus this approach requires astute dynamic modelling (and adequate model reduction in case of a complex system), but since a cart pole system is a relatively simple system, we can use this approach. A PD controller is used (which takes the feedback of both the generalised coordinates and their first time derivatives).

The control architecture is given below:

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/MBCArch.jpg)

The control objective is **Regulation** and disturbance rejection is also explored. For now, only the case of double actuation (both cart and pole actuated) is explored, but MBC for single actuation system (only cart actuated) is also possible. Refer to the branch `SingleAct` for the codes related to single actuation, but as of now, they are wrong and giving garbage results.

**Two methodologies of coding the dynmaics are used**:

`[t,w1] = ode45(@(t,w1)ClosedLoopDynamics_1(t,w1,M_mat,K_mat,A,B,Kp,Kd,dist,dof),t_span,w_0);`

`[t,w2] = ode45(@(t,w2)ClosedLoopDynamics_2(t,w2,M_mat,Kp,Kd,dist,dof),t_span,w_0)`

**Both should and give the same results**.

Distrubance rejection is tested for various kinds of disturbance forces on the cart:

* *Harmonic Disturbance*: 

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/CartActuatorEffort_PDController_BothActuated_HarmonicDisturbance.png)
![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/PoleActuatorEffort_PDController_BothActuated_HarmonicDisturbance.png)
![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/Response_PDController_BothActuated_HarmonicDisturbance.png)


* *Impulse Disturbance*:

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/CartActuatorEffort_PDController_BothActuated_ImpulseDisturbance.png)
![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/PoleActuatorEffort_PDController_BothActuated_ImpulseDisturbance.png)
![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/Response_PDController_BothActuated_ImpulseDisturbance.png)

* *Static Disturbance*: 

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/CartActuatorEffort_PDController_BothActuated_StaticDisturbance.png)
![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/PoleActuatorEffort_PDController_BothActuated_StaticDisturbance.png)
![sys](https://github.com/average-engineer/ModelBasedControl_CartPole/blob/main/Images/Response_PDController_BothActuated_StaticDisturbance.png)

As observed in the case of a static disturbance force, there exists a steady state error which reflects the need for an integral controller too.
