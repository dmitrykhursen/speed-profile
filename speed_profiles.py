import numpy as np
# import torch

#################################################
#
# topic: Speed profile for autonomous student formula 
# author: Dmytro Khursenko
# e-mail: khursdmy@fel.cvut.cz
# date: January, 2023
#
#################################################


# TODO:
# torch code to numpy
# why: return segment_curvature + 0.0001?
# add constant, optimal, and reactive speed profiles
# ? REDO lengths and curvatures
# for s in range(start_idx, first_pass.shape[0]):   # ??? start from 0 or 1 ???
#
# if initial_speed != 0:    # ??? Do I need this?? 
            #     second_pass[0] = torch.minimum(
            #                                     first_pass[0],
            #                                     torch.sqrt(initial_speed ** 2 + 2 * MAX_ACCELERATION * seglengths[0])
            #                                   )



# constants figuring in friction ellipse
CAR_MU = 0.8
GRAVITY = 9.8
MAX_ACCELERATION = 2    # TODO: ask for better values
MAX_BRAKING = 4       # this is actually F on the left side, F = m*a where m is the weight of car

class StanfordProfile():
    """ Three-pass speed profile. The algorithm is taken from PhD thesis by Nitin R. Kapania """

    def __init__(self):
        self.safe_max_speed = 5.75 # precomputed using corner speed formula: v=sqrt(mu*(m*g+F_aero)*r/m)  
        self.constant_speed = 5 # 5 m/s is the old speed profile
    
    def curvatures(self, path):
        # returns an approximation of path curvature using inscribed circles

        def curvature(origin, current, destination):
            # calculates radius of circle on whose circumference lie points origin, current and destination
            a = np.linalg.norm(current     - origin )
            b = np.linalg.norm(destination - current)
            c = np.linalg.norm(destination - origin )
            
            q = (a ** 2 + b ** 2 - c ** 2) / (2 * a * b)
            return (2 * np.sqrt(1 - q**2)) / c


        segment_curvature = np.zeros((path.shape[0], ))

        for i in range(path.shape[0]):
            segment_curvature[i] = curvature(path[i-1, :], path[i, :], path[(i+1) % path.shape[0], :])
        
        return segment_curvature + 0.0001 # the 1/10000 hack which just might save my thesis

    def lengths(self, path):
        # compute segment length of a path

        segment_length = np.zeros(len(path))

        for i in range(len(segment_length)):
            # Puts the length of segment between i-th and (i+1)-th path point at position i
            segment_length[i - 1] = np.linalg.norm(path[i, :] - path[i - 1, :])

        return segment_length
    
    def compute_speed_profile(self, path, initial_speed):
        segcurvatures = self.curvatures(path)
        seglengths = self.lengths(path)

        def first_pass():
            # returns speed at the edge of the car's friction ellipse
            ret = np.sqrt((CAR_MU * GRAVITY) / segcurvatures )

            return ret

        def second_pass(first_pass):
            # modifies speed from first pass so that no too extreme acceleration is done
            if initial_speed is None:
                # second_pass = first_pass.clone() # autograd requires access to original profile tensor, so no inplace ops
                second_pass = np.copy(first_pass) 

            else:
                # second_pass = torch.tensor([initial_speed] + [0.] * (len(first_pass) - 1), dtype=torch.float64, requires_grad=False)
                second_pass = np.array([initial_speed] + [0.] * (len(first_pass) - 1), dtype=float)

            for s in range(1, first_pass.shape[0]):
                compensation = 2 * MAX_ACCELERATION * seglengths[s-1]
                # remove clone() and autograd FUCKING DIES!!!
                second_pass[s] = np.minimum(
                                                first_pass[s],
                                                np.sqrt(second_pass[s-1] ** 2 + compensation)
                                              )

            return second_pass
    
        def third_pass(second_pass):
            # modifies speed from second pass so that no too extreme braking is done
            # third_pass = second_pass.clone() # maybe empty_like(second_pass) and third_pass[0] = second_pass[0] would be faster
            third_pass = np.copy(second_pass) # maybe empty_like(second_pass) and third_pass[0] = second_pass[0] would be faster


            for s in range(second_pass.shape[0] - 1, 0, -1): # the range should be up to 0!!! (to -1 after changes the last index, but I don't want that!)
                compensation = 2 * MAX_BRAKING * seglengths[s]
                third_pass[s-1] = np.minimum(
                                                second_pass[s-1],
                                                np.sqrt(third_pass[s] ** 2 + compensation)
                                               )
            return third_pass
    

        first  = first_pass()
        second = second_pass(first)
        third  = third_pass(second)

        # print("Phases:")
        # print(first)
        # print(second)
        # print(third)
    
        return third 


if __name__ == '__main__':
    path = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]])
    initial_speed = 0

    stanford_sp = StanfordProfile()
    speeds = stanford_sp.compute_speed_profile(path, initial_speed)

    print("Speeds:", speeds)

