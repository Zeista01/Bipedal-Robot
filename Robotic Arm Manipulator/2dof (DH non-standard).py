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
    
    def forward_kinematics(self, thetas):
        
        l1, l2 = self.link_lengths
        t1, t2 = thetas
        
        A1 = self.dh_transform(t1, 0, l1, 0)
        A2 = self.dh_transform(t2, 0, l2, 0)
        
        T1 = A1
        T2 = T1 @ A2
        
        o0 = np.array([0, 0, 0, 1])
        o1 = T1 @ np.array([0, 0, 0, 1])
        o2 = T2 @ np.array([0, 0, 0, 1])
        
        return [o0, o1, o2]
    
    def plot(self, thetas):
        
        positions = self.forward_kinematics(thetas)
        
        xs = [pos[0] for pos in positions]
        ys = [pos[1] for pos in positions]
        
        plt.figure()
        plt.plot(xs, ys, 'bo-', linewidth=2, markersize=10)
        plt.xlim(-sum(self.link_lengths), sum(self.link_lengths))
        plt.ylim(-sum(self.link_lengths), sum(self.link_lengths))
        plt.xlabel('X axis')
        plt.ylabel('Y axis')
        plt.title('2DOF Robotic Manipulator')
        plt.grid(True)
        plt.show()

link_lengths = [1, 1]  
thetas = [np.pi/4, np.pi/4]  

manipulator = TwoDOFManipulator(link_lengths)
manipulator.plot(thetas)
