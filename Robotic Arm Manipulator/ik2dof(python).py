import numpy as np
import matplotlib.pyplot as plt

class TwoDOFManipulator:
    def __init__(self, link_lengths):
        self.link_lengths = link_lengths
    
    def dh_transform(self, theta, d, a, alpha):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [ 0,     sa,     ca,    d],
            [ 0,      0,      0,    1]
        ])
    
    
    def inverse_kinematics(self, target):
        l1, l2 = self.link_lengths
        x, y = target
        
        cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        sin_theta2 = np.sqrt(1 - cos_theta2**2)
        theta2 = np.arctan2(sin_theta2, cos_theta2)
        
        k1 = l1 + l2 * cos_theta2
        k2 = l2 * sin_theta2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        return [theta1, theta2]
    

link_lengths = [1, 1]

manipulator = TwoDOFManipulator(link_lengths)

target_position = [2, 0] 
ik_thetas = manipulator.inverse_kinematics(target_position)
print(f"theta1 and theta2 are: {ik_thetas}")

