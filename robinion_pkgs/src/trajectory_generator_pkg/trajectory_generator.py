import numpy as np

class TrajectoryGenerator():
    def __init__(self):
        super().__init__()

    def test(self):
        print("Import success")
        
    def minimum_jerk(self, xi, xf, d=0.5, dt=0.01):
        list_x = []
        t = 0.0
        while t < d:
            x = xi + (xf-xi) * (10*(t/d)**3 - 15*(t/d)**4 + 6*(t/d)**5)
            list_x.append(x)
            t += dt
        return np.array(list_x)
    
    def bezier_curve(self):
        pass