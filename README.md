# 3R_motion_Scilab

3R motion with Scilab
See the TriVex SL/SLi vertical pack loader using two 3R arm robots: https://youtu.be/58p0063SkLE
Using the functions developed at class, finish the Scilab code to generate the vertical motion of one of the robots. The code should generate the following results:
1 - Plot of the 3R motion
2 - Plot of joint speeds versus time. For this, you can use the following sequence at each step of the motion:
- Compute the pose of the end-effector
- Solve the inverse position analysis to obtain the configuration of the robot and, in particular, the position of each joint
- Compute the jacobian at this configuration
- Obtain joint speeds by multiplying the jacobian with the required end-effector twist (joint speeds = J·T)
Optionally (as bonus) you can generate the following results:
3 - Plot the torque that J1 must sustain versus time (you can consider the weight of L1 and L2 at the middle of the link, and the weight of the end-effector at the middle
of g3)
4 - In the function that solves the inverse position analysis of the robot, add a check to guarantee that the inputs are inside the workspace of the robot


All the code needed to run the simulation is provided in the 3R1.sce file. Just click play and the simulation will start.

The file iniciates with the declaration of all the constants and parameters used.

First of all we defined all the functions needed, as the kinematics functions, wich provide us the position of the joints. Dinamic functions also, wich allow us to compute the joint speed with the jacobian and the twist. We need alfo functions to represent the motion of the joints and links and the evolution of w1,w2 and w3 versus time.

The main program that simulate the motion is divided in two loops, one for down-move and another for up-move. The first one starts at initial position and stops on the final position of the end-effector. For each turn of the loop we use the functions to find the joint position and consequently the jacobian to represent the motion and the jont speed.
The second loop is identhical to the first one, but it goes from the final position to the inicial position of the end-effector. 

Some parameters, as the velocity (v) and the number of iterations (n_iter) can be tunned by the user just changing it's values on the parameter declaration section.
