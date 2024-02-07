import env
from param import Parameter as p
import plot
import GenerateInitialPath
import numpy as np
import util
import matplotlib.pyplot as plt
import bezier

middle_path =[[1.5, 5.502545454545454], 
              [2.350356732570509, 4.720234724421152], 
              [3.2234349554925092, 3.9419636351438645], 
              [4.139034854331049, 3.178655318696748], 
              [5.128898801984397, 2.4413046878567], 
              [6.641710968830737, 1.5035645582636796], 
              [7.737276661568834, 1.1390684762300192], 
              [8.838213145077459, 0.8611157500365769], 
              [9.943723625522994, 0.6457570029141132], 
              [11.049658977908072, 0.4768015370926607], 
              [12.15576494448705, 0.34085825317756757], 
              [13.261774331346047, 0.22690719559914144], 
              [14.370036674940698, 0.12509464347774618], 
              [15.477549305287203, 0.026920525201214057], 
              [16.588492671249526, -0.07656568693277714],
              [17.70183737008911, -0.1943922128402597], 
              [18.82009838641799, -0.33697268774248224], 
              [19.94581273218211, -0.5168505587677311], 
              [21.085014935458226, -0.7510425929387718], 
              [22.24415307987719, -1.064595753584901], 
              [23.44938796023403, -1.5048898757429405], 
              [24.861895899655323, -2.242083288023167], 
              [25.42844417328579, -2.6886262961933616], 
              [25.971882838144108, -3.1448970979665125], 
              [26.497652149006935, -3.6087505672764495], 
              [27.008657447980287, -4.076992303163212], 
              [27.509731335841924, -4.548894076266217], 
              [28.00557333278683, -5.024291820772816], 
              [28.5, -5.502545454545454]]

p.N = len(middle_path)

cubicX, cubicY = GenerateInitialPath.cubic_spline_by_waypoint(middle_path)
theta, phi, v = np.array([0]*p.N), np.array([0]*p.N), np.array([0]*p.N)
trajectory_matrix = np.array([cubicX, cubicY, theta, phi, v])
trajectory_vector = util.matrix_to_vector(trajectory_matrix)
plot.vis_path(trajectory_vector)

theta = GenerateInitialPath.generate_initialtheta(middle_path)
plt.plot(theta)
plt.show()

CR = GenerateInitialPath.generate_curvature_radius(middle_path)
plt.plot(CR)
plt.show()

print(len(theta))

p0 = [0, 0]
start_theta = np.pi/4
p1_x, p1_y = [], []
min_CR_list = []
error_list = []
for i in range(len(theta)):
    p2 = [cubicX[i], cubicY[i]]
    goal_theta = theta[i]
    p1 = bezier.calc_p1(p0, p2, start_theta, goal_theta)
    p1_x.append(p1[0])
    p1_y.append(p1[1])
    min_t, min_CR = bezier.minimum_curvature_radius(p0, p1, p2)
    min_CR_list.append(min_CR)
    bezier_x, bezier_y = bezier.generate_bezier(p0, p1, p2)
    bezier_k = bezier.calc_curvature(p0, p1, p2)
    error = (bezier_k[-1] - 1/CR[i]) ** 2
    error_list.append(error)
    bezier_theta = bezier.calc_theta(p0, p1, p2)
    print(bezier_theta[-1], theta[i])
    #plot.test_path(cubicX, cubicY, bezier_x, bezier_y)
    #plt.show()


check_min_CR = []
check_error = []
lower_limit_R = p.L/np.tan(p.phi_max)
error_upper_limit = 1
for R in min_CR_list:
    if R > lower_limit_R:
        check_min_CR.append('True')
    else:
        check_min_CR.append('False')

print(check_min_CR)
plt.plot(min_CR_list)
plt.show()

for error in error_list:
    if error < error_upper_limit:
        check_error.append('True')
    else:
        check_error.append('False')
        
print(check_error)
plt.plot(error_list)
plt.show()

