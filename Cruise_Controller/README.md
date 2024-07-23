# PID-controller-for-cruise-control
Developing a PID controller in Python for Cruise Control of a car for a fixed set-point velocity.

                                                          ** Assignment**

The model of the cruise control system is relatively simple. If the inertia of the wheels is neglected, and it is assumed that air drag (which is proportional to the car’s speed at low speeds) is what is opposing the motion of the car, then the problem is reduced to a simple first order system.
Thus the motion of the car can be written as,

mv’ + bv = u

where u is the input force provided by the engine to move the car at a certain velocity. Though it is intuitive to think of an input as something related to the gas pedal, but unfortunately to represent such a system accurately the dynamics of it becomes really complicated for a task of a beginner level. So for now we will just assume that our engine can give us a certain amount of force without actually looking how that force translates to real life situations.

                                                             **Task**

1.	Define your constant parameters for the car i.e. mass and the drag coefficient.
2.	Find v and ˙v from the dynamical model and compare it with the set point values.
 
3.	Find the error and use the PID controller with appropriate tuning parameters to tune the controller to an acceptable level of accuracy.
4.	Make sure that the tuning parameters you choose give the system a rise time of about 10 secs and maximum overshoot of less than 5%. Remember that rise time is the time it takes to reach 90% of the steady state value.
5.	Plot the results in a graph showing the set point change and the response of the system to the change.
                                                                                             
