import numpy as np
import os
import json
import matplotlib.pyplot as plt

from scipy.interpolate import UnivariateSpline


class PathPlanning:

    def __init__(self, debugging=False):

        self.sorted_blue_cones = []
        self.sorted_yellow_cones = []
        self.artificial_cones = []

        self.start_points = [np.array([0., 0.])]

        self.filling_cones_distance = 3.5
        self.distances_between_b_and_y = []

        # parameters of the line
        self.k = 0
        self.c = 0

        self.path_distance = 0

        self.debugging = debugging
        if self.debugging:
            self.ks = []
            self.cs = []
    
    def sort_points(self, points, start):
            # sort points by distance
            sorted_points = sorted(points, key=lambda x: np.linalg.norm(x - start))
            sorted_points = np.reshape(sorted_points, (points.shape))

            return sorted_points
    
    def sort_cones(self, cones, blue):
        # sort cones in a right sequence

        sorted_cones = []         
        start = self.start_points[0]

        for i in range (len(cones)):
            cones = self.sort_points(cones, start) # sort cones according to the current start point
            closest_cone = cones[0]
            cones = np.delete(cones, 0, axis=0)

            # ??? do not understand
            # if blue and len(self.sorted_blue_cones) > 0 \
            #     and np.linalg.norm(self.sorted_blue_cones[0]) > np.linalg.norm(closest_cone):
            #         continue
            # elif not blue and len(self.sorted_yellow_cones) > 0 \
            #     and np.linalg.norm(self.sorted_yellow_cones[0]) > np.linalg.norm(closest_cone):
            #         continue

            sorted_cones.append(closest_cone)
            start = closest_cone

        return np.array(sorted_cones)

    def filter_cones(self, cones):
        # filter cones accordnig to the rules:
        # 1. the distance between blue or yellow cones is up to 6m

        # REDO? filter only by max distance? max 20m?; between same cones - distance is up to 6m
        # TODO: set min dist 1m?
        # optimize the func using numpy arr operations

        def compute_distances_between_cones(cones):
            distances = [np.linalg.norm(cones[0])] # add the distance from start (0,0) to the first cone

            for i in range(len(cones)-1):
                distances.append(np.linalg.norm(cones[i+1] - cones[i]))

            return distances

        distances = compute_distances_between_cones(cones)

        filtered_cones = []

        min_dist = 1. #0.5
        max_dist = 6.

        print("distances:")
        print(distances)

        for i in range(len(distances)):
            if distances[i] >= min_dist and distances[i] <= max_dist:
                filtered_cones.append(cones[i])
            elif i == 0 and distances[i] <= 8:  # is the condition for the first cones needed?
                    filtered_cones.append(cones[i])
            else:
                break
                
        return np.array(filtered_cones)

    def find_line_parameters(self, pointB, pointY):
        if pointB[0]-pointY[0] != 0:
            k = (pointB[1]-pointY[1])/(pointB[0]-pointY[0]) # old version x vs y .....
            # k = (pointB[0]-pointY[0])/(pointB[1]-pointY[1])

        else:
            k = self.k

        self.k = k
        c = pointB[1] - self.k*pointB[0]
        self.c = c

        # if self.debugging:
        #     self.ks.append(self.k)
        #     self.cs.append(self.c)
    

    def points_above_the_line(self, points):
        points = np.array([p for p in points if not self.is_already_added(p)]) # filter used points
        if len(points) == 0:
            return np.array([])
        else:   
            return points[np.sign((points[:, 1] - self.k * points[:, 0] - self.c)) == np.sign(self.sorted_yellow_cones[-1][0]-self.sorted_blue_cones[-1][0])]

    def find_blue_and_yellow_cone(self, B_points, Y_points, step):
        sorted_B_points = self.sort_points(B_points, self.start_points[-1])
        sorted_Y_points = self.sort_points(Y_points, self.start_points[-1])
        # print("B_sorted: ", sorted_B_points)
        # print("Y_sorted: ", sorted_Y_points)

        ###
        if len(sorted_Y_points) == 0 and len(sorted_B_points) == 0:
            return None, None

        if len(sorted_Y_points) == 0:
            # instead of fill missing
            if len(self.sorted_yellow_cones) == 0:
                return None, None

            b = sorted_B_points[0]
            y = self.sorted_yellow_cones[-1]
            return b, y

        if len(sorted_B_points) == 0:
            if len(self.sorted_blue_cones) == 0:
                return None, None

            b = self.sorted_blue_cones[-1]
            y = sorted_Y_points[0]
            return b, y

        if step == 1:
            b = sorted_B_points[0]
            y = sorted_Y_points[0]
            # self.check_cones_side(b, y)
            return b, y
        
        dist_to_blue_cone = np.linalg.norm(sorted_B_points[0] - self.start_points[-1])
        dist_to_yellow_cone = np.linalg.norm(sorted_Y_points[0] - self.start_points[-1])
        # print("dist to blue: ", dist_to_blue_cone, " dist to yellow: ", dist_to_yellow_cone)

        if dist_to_blue_cone < dist_to_yellow_cone:
            b = sorted_B_points[0]
            y = self.sorted_yellow_cones[-1]
        else:
            b = self.sorted_blue_cones[-1]
            y = sorted_Y_points[0]

        return b, y
    
    def is_new_center_ok(self, new_center):
        vector_1 = np.array([self.start_points[-2][0] - self.start_points[-1][0],
                             self.start_points[-2][1] - self.start_points[-1][1]])
        vector_2 = np.array([new_center[0] - self.start_points[-1][0],
                             new_center[1] - self.start_points[-1][1]])

        angle_1 = self.compute_angle_between_vectors(vector_1, vector_2)
        # print("new center: ", new_center , " print angle ", degrees(angle_1))
        angle_2 = 2*np.pi - angle_1
        
        # print(degrees(angle_1), degrees(angle_2))

        # if(250*np.pi/180 > angle_1 and 250*np.pi/180 > angle_2): # 270 was before
        if(235*np.pi/180 > angle_1 and 235*np.pi/180 > angle_2): # 250 was before
            return True
        else:
            return False
    
    def calculate_center(self, pointB, pointY):
        return np.array([(pointB[0]-pointY[0])/2 + pointY[0], (pointB[1]-pointY[1])/2 + pointY[1]])
    
    def is_already_added(self, point):
        return (np.all(np.isin(point, self.sorted_yellow_cones)) or np.all(np.isin(point, self.sorted_blue_cones)))
    
    def return_stack(self, object_name):
        if object_name == "yellow cones":
            return np.vstack(self.sorted_yellow_cones)
        elif object_name == "blue cones":
            return np.vstack(self.sorted_blue_cones)
        elif object_name == "artificial cones":
            if len(self.artificial_cones) == 0:
                return []
            return np.vstack(self.artificial_cones)
        elif object_name == "centers":
            return np.vstack(self.start_points)
    
    def compute_angle_between_vectors(self, vector_1, vector_2):
        # print("vectro_1: ", vector_1, " vector_2: ", vector_2)
        if all(vector_1 == [0, 0]) or all(vector_2 == [0, 0]):
            return 0

        unit_vector1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector2 = vector_2 / np.linalg.norm(vector_2)

        dot_product = np.dot(unit_vector1, unit_vector2)
        # print("dot product: ", dot_product)

        if dot_product < -1.0:
            dot_product = -1
        elif dot_product > 1.0:
            dot_product = 1 
        
        # print("dot product: ", dot_product)

        angle = np.arccos(dot_product)  # angle in radians

        return angle
    
    def find_path(self, B, Y, n_steps=0, verbose=False):

        if n_steps < 0:
            if verbose:
                print("ERROR: n_steps is negative")

            return self.return_stack(object_name="centers")

        if verbose:
            print("raw B,Y:")
            print(B)
            print(Y)

        step = 1
        B = self.sort_cones(B, True)
        Y = self.sort_cones(Y, False)

        # raw_B = np.copy(B)
        # raw_Y = np.copy(Y)

        if verbose:
            print("sorted B,Y:")
            print(B)
            print(Y)

        # print(self.blue_distance, self.yellow_distance)

        if len(B) != 0:    
            B = self.filter_cones(B)
        if len(Y) != 0:
            Y = self.filter_cones(Y)
        
        if verbose:
            print("filtered B,Y:")
            print(B)
            print(Y)
        
        # if B or Y are empty, fill artificial cones ?
        if len(B) == 0 or len(Y) == 0:
            print("Len B == 0 or Len Y == 0")
            return self.return_stack(object_name="centers")

            
        while True:
            if verbose:
                print("Step ", step)
            if step == n_steps:
                if verbose:
                    print("Path algorithm was stopped: step == n_steps")
                break

            if step != 1:
                self.find_line_parameters(self.sorted_blue_cones[-1], self.sorted_yellow_cones[-1])
                
                B_hat = self.points_above_the_line(B)
                Y_hat = self.points_above_the_line(Y)
                
                # if len(B_hat) == 0 and len(Y_hat) == 0: #old
                if len(B_hat) == 0 or len(Y_hat) == 0:
                    if verbose:
                        print("Path algorithm was stopped: B_hat.size == 0 and Y_hat.size == 0")
                    
                    return self.return_stack(object_name="centers")
                    # break

                # if len(B_hat) == 0:

            #         # print("B_hat is empty")
                    
            #         if len(Y_hat) > 1:
            #             if len(Y_hat) > 2: # ? > 1
            #                 full_Y = Y_hat[0:3, :]
            #             else:
            #                 full_Y = Y_hat

            #             B_hat, _ = self.fill_missing(B_hat, full_Y)

            #             for b_c in B_hat:
            #                 self.artificial_cones.append(b_c)
                        
            #             # B = np.vstack((B, B_hat))
            #             if len(raw_B) == 0:
            #                 raw_B = raw_B.reshape(-1,Y_hat.shape[1])
            #             B = np.vstack((raw_B, B_hat))

            #             B = self.sort_cones(B, True)
            #             B = self.filter_cones(B, self.blue_distance)
            #             B_hat = self.points_above_the_line(B) # recalculate because some artificial points can be placed before than already used points

            #             # for b_c in B_hat:
            #             #     if np.all(np.isin(b_c, B)):
            #             #         self.artificial_cones.append(b_c)

            #     elif len(Y_hat) == 0: 
            #         # print("Y_hat is empty")

            #         if len(B_hat) > 1:
            #             if len(B_hat) > 2: # ? > 1
            #                 full_B = B_hat[0:3, :]
            #             else:
            #                 full_B = B_hat
            #             _, Y_hat = self.fill_missing(full_B, Y_hat)

            #             for y_c in Y_hat:
            #                 self.artificial_cones.append(y_c)
                            
            #             # Y = np.vstack((Y, Y_hat))
                        
            #             if len(raw_Y) == 0:
            #                 raw_Y = raw_Y.reshape(-1,Y_hat.shape[1])
            #             Y = np.vstack((raw_Y, Y_hat))

            #             Y = self.sort_cones(Y, False)
            #             Y = self.filter_cones(Y, self.yellow_distance)
            #             Y_hat = self.points_above_the_line(Y)

            else:
                B_hat = B
                Y_hat = Y


            if verbose:
                print(B_hat)
                print(Y_hat)

            b, y = self.find_blue_and_yellow_cone(B_hat, Y_hat, step)

            if verbose:
                print("decided: b: ",b," y: ",y)
                
            if b is None or y is None:
                if verbose:
                    print("Path algorithm was stopped: b_cone or y_cone is None")
                break

            if self.is_already_added(b) and self.is_already_added(y):
                if verbose:
                    print("The same b and y. Stop")
                break

            if not self.is_already_added(b):
                self.sorted_blue_cones.append(b)
            if not self.is_already_added(y):
                self.sorted_yellow_cones.append(y)
        
            center = self.calculate_center(b, y)   

            dist_to_next_center = np.linalg.norm(center - self.start_points[-1])
            # if np.linalg.norm(center - self.start_points[0]) > 20: 
            if self.path_distance + dist_to_next_center > 20: 
                # the planned path is far long
                if verbose:
                    print("The path is planned far enough")
                break

            if len(self.start_points) > 3 and not self.is_new_center_ok(center): # > 3 is better but > 2 is "smarter"
                # heading vector wants to go backwards
                if verbose:
                    print("The wrong place of a new center")
                break


            self.start_points.append(center)

            dist_between_b_and_y = np.linalg.norm(b-y)
            self.distances_between_b_and_y.append(dist_between_b_and_y)
            self.path_distance += dist_to_next_center
            if verbose:
                print("b_cone: ", b, " y_cone: ",y, " center: ",center, " path distance: ", self.path_distance, " dist between b and y: ", dist_between_b_and_y)
            if verbose:
                print("Step ", step, " done!")
                print()
            step += 1

        return self.return_stack(object_name="centers")
    
    def smooth_the_path(self, path):
        print("here ---")
        print(path)

        if len(path) < 4:
            return path

        x = path[:,0] 
        y = path[:,1]

        spl = UnivariateSpline(x, y)

        new_y = np.array(spl.__call__(x))
        print("new y:")
        print(new_y)
        new_path = np.stack((x, new_y), axis=1)
        print(new_path)
        print(np.shape(new_path))
        return new_path


def open_file_for_debug(filepath, filename, wheel_speed):
        with open(filepath+filename) as file:
            data = json.load(file)
    
        return data


def plot_results(B_cones, Y_cones, path, b_bound, y_bound):
    fig = plt.figure(1, figsize=(20, 10))
    axe = fig.add_axes([0.05, 0.05, 0.9, 0.9])
    axe.set_aspect("equal")

    if len(B_cones > 0):
        axe.plot(B_cones[:, 0], B_cones[:, 1], "o", color="blue", mec ="black")
    if len(Y_cones > 0):
        axe.plot(Y_cones[:, 0], Y_cones[:, 1], "o", color="yellow", mec ="black")
    
    axe.plot(path[:, 0], path[:, 1], "o", color="red")
        
    axe.plot(b_bound[:, 0], b_bound[:, 1], color="blue")
    axe.plot(y_bound[:, 0], y_bound[:, 1], color="yellow")
    axe.plot(path[:, 0], path[:, 1], color="red")

    plt.show()

def mirrorImage(a, b, c, x1, y1):
    # source: https://www.geeksforgeeks.org/find-mirror-image-point-2-d-plane/
    temp = -2 * (a * x1 + b * y1 + c) /(a * a + b * b)
    x = temp * a + x1
    y = temp * b + y1
    return x, y
    

if __name__ == '__main__':

    print("Hello world!")

    filepath = os.getcwd() + "/data/" + "trackdrive_sim/"
    filestarts = "trackdive_ugly_"
    init_speed = 0 #
    # speeds = []

    for i in np.arange(0.0, 50., 1.0):
        filename = filestarts + str(i) + "s" + ".json"

        # if i < 29. :
        #     continue

        if os.path.exists(filepath+filename):
            data = open_file_for_debug(filepath, filename, init_speed)
            # init_speed = speed
            # speeds.append(speed)

            # break
    
            print(data)
            print()

            cones = np.array(data["cones"])
            # path = np.array(data["path"])

            # print(cones)
            # normal situation
            B_cones = cones[cones[:, 2] == 0, :2]
            Y_cones = cones[cones[:, 2] == 1, :2]

            # for trackdrive_sim maps are wrong (blue cons should be on the left!)
            # B_cones[:, 0], B_cones[:, 1] = mirrorImage(1, 0, 0, B_cones[:, 0], B_cones[:, 1]) #
            # Y_cones[:, 0], Y_cones[:, 1] = mirrorImage(1, 0, 0, Y_cones[:, 0], Y_cones[:, 1]) #

            ### x is go forward
            print(type(B_cones))
            print(B_cones)
            # print(np.array([[B_cones[:,1]], [B_cones[:,0]]]))
            tmp = B_cones[:,0].copy()
            B_cones[:,0], B_cones[:,1] = B_cones[:,1], tmp
            tmp = Y_cones[:,0].copy()
            Y_cones[:,0], Y_cones[:,1] = Y_cones[:,1], tmp

            print(B_cones)
            print(Y_cones)




            # is it beeter to work with y as a go forward axis or x is a forward axis??? 

            path_planner = PathPlanning()
            path = path_planner.find_path(B_cones, Y_cones, verbose=True)
            print(path)

            # path = np.array([[0., 0.]])
            b_boundaries = path_planner.return_stack("blue cones")
            y_boundaries = path_planner.return_stack("yellow cones")
            # plot_results(B_cones, Y_cones, path, b_boundaries, y_boundaries)

            new_path = path_planner.smooth_the_path(path)
            plot_results(B_cones, Y_cones, new_path, b_boundaries, y_boundaries)

            # break

