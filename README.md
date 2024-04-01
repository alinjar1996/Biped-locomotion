# Biped-locomotion
This repository contains code of an 8 DoF biped locomotion on a flat ground. The dynamic formulation is done using  Dual Algebra and Decoupled Natural Orthogonal Complement (DeNOC) matrices.

The uniqueness of the mathematical framework is that it gives the equations of motion of a  biped system along with the reaction forces along the joint axes, defined as the Joint Axis Reaction Force (JARF) framework, without using the conventional approach of breaking a kinematic chain to obtain the expressions of constraint force(s).

The unified dual algebra framework to obtain equations of motion and JARF simultaneously can help one to design biped suitable for locomotions. The equations of motion give the information on the required input torques for the locomotion, which helps decide the required torque capacity of the actuators. The equations used to obtain JARF can help choose the necessary thrust bearings in the gearbox of the actuators. Obtaining both pieces of information simultaneously before experimental validation is an advantage.

