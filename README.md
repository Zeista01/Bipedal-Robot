# Bipedal Robot
## Objective
This project aims to develop a bipedal robot, a two-legged robotic system designed to replicate human gait dynamics. Here, the robot’s movement is controlled by precise limb positioning through inverse kinematics, while stability is maintained by managing the center of mass during walking. This repository provides an overview of the fundamental concepts, codes and model for simulating and executing stable bipedal locomotion, with an emphasis on kinematics and balance control.

## Key Concepts and Tasks
### PID Control Theory
PID (Proportional-Integral-Derivative) control is a fundamental control theory that is used in control systems to maintain a desired output such as speed, position or temperature by continuously adjusting inputs based on feedback. A PID controller calculates the error as the difference between a desired setpoint (SP) and a measured process variable (PV). It then applies a correction based on three components: proportional, integral, and derivative terms.

- **Proportional (P):** The proportional term responds to the current error. It provides an output that is directly proportional to the error, allowing the system to correct deviations quickly.

- **Integral (I):** The integral term accounts for the accumulation of past errors. It helps to eliminate small errors that may not be fully corrected by the proportional term alone, achieving precise control over time.

- **Derivative (D):** The derivative part predicts future error by considering the rate of change of the error. It reduces overshoot and prevents oscillations by slowing down the system as it approaches the desired value.

### Task 1: Cruise Controller 
#### Aim
To develop a Cruise Controller using Python and Matlab(simulink) for maintaining desired set point velocity of a car.
#### Procedure
In Python
1. Set the mass of the vehicle(m) and the drag coefficient(b).
2. Define the set-point velocity(v=20 m/s).
3. Implemente a PID controller in Python.
4. Determine the transfer function of the system.

In Simulink
1. Build the circuit of the cruise controller using the PID controller in Simulink.
2. Initially, set the integral gain (Ki) and derivative gain (Kd) to zero and tune the proportional gain (Kp).
3. After tuning Kp, tune the values of Ki and Kd to refine the controller's performance.

**Simulink circuit design:**

![Screenshot 2024-11-07 235542](https://github.com/user-attachments/assets/b502fb53-0f23-4376-9cfc-54237ecd8016)

#### Result

The PID controller was successfully tuned to achieve:
- A rise time of approximately 10 seconds.
- A maximum overshoot of less than 5%.

**Python result:**
![SyKU5ti5C](https://github.com/user-attachments/assets/12fbe362-feef-4fc4-8daf-3996e7bcd4f1)

**Simulink result:**
![Screenshot 2024-11-07 235517](https://github.com/user-attachments/assets/a90c50b4-ed2a-40e9-8107-4ad6e709757c)

---

### Mechanics and Control of Robotic Manipulators

Watched the YouTube playlist on Mechanics and Control of Robotic Manipulators: <br>
https://www.youtube.com/playlist?list=PLyqSpQzTE6M-tWPjnJjFo9sHGWxgCnGrh


In bipedal robots, kinematics plays a crucial role in defining and controlling movement. Kinematics is divided into forward kinematics and inverse kinematics, both of which are essential for achieving precise and coordinated motion in robotic manipulators.

- **Forward Kinematics:** Forward kinematics involves calculating the robot's end-effector position based on given joint angles. In a bipedal robot, forward kinematics is used to determine the exact position of each leg segment when joint angles are set, enabling accurate placement of the feet during walking.

- **Inverse Kinematics:** Inverse kinematics is the reverse process, where the desired position of the end-effector (such as the foot) is specified, and the necessary joint angles are calculated to achieve that position. This is particularly important in bipedal robots for defining precise leg trajectories that maintain stability and allow for smooth, balanced walking motions.

### Task 2: Finding FK/IK of Manipulator arm in XY Plane in Python Using matplotlib 
#### FORWARD KINEMATICS (FK)
1. **Construct the DH Table**:
    - Defined the Denavit-Hartenberg (DH) parameters for the manipulator.
    - Used these parameters to create the DH transformation matrix.
2. **Implement the DH Transformation Matrix**:
    - Inputted the DH transformation matrix in Python.
3. **Develop the Forward Kinematics Function**:
    - Created a function that sequentially multiplies transformation matrices to compute the end-effector position.
    - Post-multiplied the matrices according to the manipulator's configuration.
4. **Define FK Parameters**:
    - Set up the necessary parameters for forward kinematics.
    - Inputted the specific values corresponding to the desired assembly configuration.
   
#### Result
- Successfully defined the DH parameters and transformation matrices.
- The forward kinematics function accurately calculates the end-effector position for the specified assembly.

![WhatsApp Video 2024-11-08 at 2 56 41 PM](https://github.com/user-attachments/assets/bbcc3a39-8b33-4206-93d2-557aeead6b1e)

![WhatsApp Video 2024-11-08 at 5 23 16 PM](https://github.com/user-attachments/assets/70865eed-6e1d-4e9a-b3c7-0176f15c5572)

#### INVERSE KINEMATICS (IK)
1. **Construct the DH Table**:
    - Defined the Denavit-Hartenberg (DH) parameters for the manipulator.
    - Used these parameters to create the DH transformation matrix.
2. **Implement the DH Transformation Matrix**:
    - Inputted the DH transformation matrix in Python.
3. **Develop the Inverse Kinematics Function**:
    - Create a function to calculate the relative inverse kinematics (IK) parameters for the desired assembly configuration.

#### Result
- Successfully defined the DH parameters and transformation matrices.
- The inverse kinematics function accurately calculates the necessary joint angles for the specified end-effector position.

![WhatsApp Video 2024-11-08 at 3 51 17 PM](https://github.com/user-attachments/assets/41cdd9e7-82ab-4d8d-9c85-c435e8f71b97)

![WhatsApp Video 2024-11-08 at 5 23 24 PM](https://github.com/user-attachments/assets/6d017b42-6ecf-463b-bfee-0a550df742df)

### Task 3: Simulating a Manipulator Arm in XYZ Plane in Python Using matplotlib 
#### PROCEDURE
**Choose a Method for Simulation:**

We can use one of the following approaches:
-   Denavit-Hartenberg (DH) Method
-   Closed-Form Solution
-   Numerical Analysis
-   Geometrical Method

1. Using the DH Method:
   - Defined the DH parameters and construct the DH transformation matrix.
   - Written the inverse kinematics (IK) function to determine the joint angles for the intended trajectory.
2. Using the Closed-Form Solution:
   - Defined the forward kinematics (FK) equations for the required trajectory, at first.
   - Then, solved these FK equations to obtain the IK parameters.
   - Later, ploted the simulation based on these parameters.
- Our task was to trace the trajectory of a sine wave and a line by 3DoF Arm manipulator using inverse kinematics. We went with the geometrical method to find the equations of the joint angles.

#### Result
- Successfully simulated the arm manipulator in the XYZ plane using the chosen method.
- The IK parameters were accurately determined, and the manipulator followed the intended trajectory in the simulation.

![By8LkHSWJx](https://github.com/user-attachments/assets/5d0eb689-a231-469b-8235-3af9d5096b50)

![BJlDkrSWkg](https://github.com/user-attachments/assets/41201e08-5798-4b3e-a81c-6a34fc4ff971)

---

### IMPLEMENTATION ON PROJECT
- We decided to make 2dof manipulator as Bipedal Robot legs.
- Formed the inverse kinematics equations and solved them on the Sinusoidal trajectory followed by retracment of Straight Line trajectory backwards.
- Here is the python pseudocode for finding ik solutions:
```python
def inverse_kinematics(x, y):
    r = sqrt(x**2 + y**2)

    if r > (a1 + a2):
        raise ValueError("Target is not reachable")

    cos_theta2 = (r**2 - a1**2 - a2**2) / (2 * a1 * a2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    sin_theta2 = sqrt(1 - cos_theta2**2)
    theta2 = atan2(sin_theta2, cos_theta2)

    theta1 = atan2(y, x) - atan2(a2 * sin(theta2), a1 + a2 * cos(theta2))

```

#### Result
- Successfully simulated the 2DoF in XY Plane according to the required trajectory.

![rJNpf4H-yx](https://github.com/user-attachments/assets/9a32321c-6504-4178-abde-647eb72cf70a)

---


### DESIGNING THE ROBOT
- We used Autodesk Fusion 360 Software to create the CAD model for our 2DoF robot.
- After creating the model for one leg, we mirrored it for the other leg.
- Here is the final model that we came up with.

![HkIrL97-yx](https://github.com/user-attachments/assets/20429521-7abb-49c0-bde2-b3560cba6a04)



---

### HARDWARE IMPLEMENTATION
We used 3 main components in our project.
1. 6 servos, with 3 in each leg. One in hip, one in knee and one in foot(to balance the COM while the robot is walking).
2. Arduino UNO to connect all electronic equipments.
3. PCA9685 with a 5V power supply to connect and power multiple servo motors easily.

**Arduino UNO to PCA9685 connection:**

![arduino-pca](https://github.com/user-attachments/assets/d62bc7e4-5902-483c-95cf-90bbe5d8cc5f)

- We assembled all the 3D-printed parts along with the hardware components and integrated both the hardware and software.
- From the previous python code for sine and line we stored the calculated joint angles at each instance into a text file and uploaded those angles into the arduino code.
- Hence the servos will rotate at the desired angle which will help the biped to trace the require trajectory and walk forward.

---

### FINAL IMPLEMENTATION

![Untitleddesign-ezgif com-video-speed](https://github.com/user-attachments/assets/e2e18482-1649-4838-ac25-0335463a5be7)
