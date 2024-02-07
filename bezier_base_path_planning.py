import env
from param import Parameter as p
import plot
import GenerateInitialPath
import numpy as np
import util
import utils
import matplotlib.pyplot as plt
import bezier
import csv

class BezierBasePathPlanning:
    curvature_threshold= np.tan(np.pi/2.1)/p.L #最大曲率
    error_curvature_threshold = 100
    
    def __init__(self, start, goal, start_theta, goal_theta, network):
        self.start = start
        self.goal = goal
        self.start_theta = start_theta
        self.goal_theta = goal_theta
        
    def kari(self, network, x, initial_theta):
        Is_curvature_threshold = []
        Is_error_curvature_threshold = []
        Is_collision = []
        list_length_bezier = []
        
        for middle_path in network:
            p.N = len(middle_path)
            #経路をspline補完し，滑らかにする
            cubicX, cubicY = GenerateInitialPath.cubic_spline_by_waypoint(middle_path)

            #middle_pathからスプライン補間を経由して経路上の姿勢を復元
            theta = GenerateInitialPath.generate_initialtheta(middle_path)

            #middle_pathからスプライン補間を経由して経路上の曲率半径を復元
            CR_middlepath = GenerateInitialPath.generate_curvature_radius(middle_path)

            p0 = x
            p1_x, p1_y = [], []
            max_curvature_list = []
            error_list = []
            length_list = []
            collision_list = []
            for i in range(len(theta)):
                #ベジエ曲線のp2に経路上のwaypointを指定
                #p2における接戦ベクトルの姿勢を設定
                p2 = [cubicX[i], cubicY[i]]
                middle_theta = theta[i]
                
                #p1を計算
                p1 = bezier.calc_p1(p0, p2, initial_theta, middle_theta)
                p1_x.append(p1[0])
                p1_y.append(p1[1])
               
                #ベジエ曲線の最小曲率半径を計算し，格納
                min_t, min_CR = bezier.minimum_curvature_radius(p0, p1, p2)
                max_curvature_list.append(1/min_CR)
                
                #ベジエ曲線の軌道を計算
                bezier_x, bezier_y = bezier.generate_bezier(p0, p1, p2)
                
                #ベジエ曲線の長さを計算し，格納
                length_bezier = bezier.calc_length_path(bezier_x, bezier_y)
                length_list.append(length_bezier)
                
                #ベジエ曲線の軌道に沿った曲率を計算
                bezier_k = bezier.calc_curvature(p0, p1, p2)
                
                #曲率の2乗誤差を計算し格納
                error = (bezier_k[-1] - 1/CR_middlepath[i]) ** 2
                error_list.append(error)
                
                #経路が障害物に衝突するかどうかを判定
                flag = utils.Is_collisionfree(bezier_x, bezier_y)
                collision_list.append(flag)
                
                #bezier_theta = bezier.calc_theta(p0, p1, p2)
                #print(bezier_theta[-1], theta[i])
                #plot.test_path(cubicX, cubicY, bezier_x, bezier_y)
                #plt.show()

            #ベジエ曲線が制約を満たすかどうかを確認するlist
            check_curvature_threshold = []
            check_error_curvature_threshold = []

            #curvatureが閾値より小さければtrue,そうでなければfalse
            for curvature in max_curvature_list:
                if curvature < self.curvature_threshold:
                    check_curvature_threshold.append(True)
                else:
                    check_curvature_threshold.append(False)

            #print(check_curvature_threshold)
            #plt.plot(max_curvature_list)
            #plt.title('ベジエ曲線の最小曲率半径',fontname="MS Gothic")
            #plt.show()

            #errorが閾値より地位避けれがtrue,そうでなければfalse
            for error in error_list:
                if error < self.error_curvature_threshold:
                    check_error_curvature_threshold.append(True)
                else:
                    check_error_curvature_threshold.append(False)
                    
            #print(check_error_curvature_threshold)
            #plt.plot(error_list)
            #plt.title('ベジエ曲線とmiddle_pathの接続における曲率の二乗誤差',fontname="MS Gothic")
            #plt.show()
            
            #check_listをappendする
            Is_curvature_threshold.append(check_curvature_threshold)
            Is_error_curvature_threshold.append(check_error_curvature_threshold)
    
            #距離のlistをappendする
            list_length_bezier.append(length_list)
            
            #collisionの情報をappend
            Is_collision.append(collision_list)
            
            
        return Is_curvature_threshold, Is_error_curvature_threshold, list_length_bezier, Is_collision
    
    
    def kari2(self, network, Is_curvature_threshold, Is_error_curvature_threshold, list_length_bezier, Is_collisoin):
        min_index_list = []
        for i in range(len(network)):
            min_length = 999999
            min_index = 99
            for j in range(len(network[i])):
                if Is_curvature_threshold[i][j] == True and Is_error_curvature_threshold[i][j] == True and Is_collisoin[i][j] == True:
                    if list_length_bezier[i][j] <= min_length:
                        min_length = list_length_bezier[i][j]
                        min_index = j
                    else:
                        pass
                else:
                    pass
            min_index_list.append(min_index)
            
        return min_index_list
    
    def kari3(self, network, min_index_list1, min_index_list2, length_list1, length_list2):
        min_length = 999999
        min_middlepath_index = 99
        for i in range(len(network)):
            if min_index_list1[i] == 99 or min_index_list2[i] == 99:
                continue
            else:
                sum_length = length_list1[i][min_index_list1[i]] + length_list2[i][min_index_list2[i]]
                if sum_length < min_length:
                    min_length = sum_length
                    min_middlepath_index = i
                else:
                    pass
            
        return min_middlepath_index
    
    
        
        
    

def main():
    #csvからnetwork情報を取得
    with open("network_rectangle.csv") as file:
        reader = csv.reader(file, quoting=csv.QUOTE_NONNUMERIC)
        trajectory_vectors = [row for row in reader]

    #csvから得られた情報からnetworkを構成
    network = []

    for trajectory_vector in trajectory_vectors:
        trajectory_vector = np.array(trajectory_vector)
        trajectory_matrix = util.vector_to_matrix(trajectory_vector)
        x, y = trajectory_matrix[0], trajectory_matrix[1]
        path = []
        for i in range(len(x)):
            path.append([x[i], y[i]])
        network.append(path)
        
    start = [-1, 3]
    goal = [32, -4]
    
    start_theta = 0
    goal_theta = 0
    
    planner = BezierBasePathPlanning(start, goal, start_theta, goal_theta, network)
    Is_curvature_threshold1, Is_error_curvature_threshold1, list_length_bezier1, Is_collision1 = planner.kari(network, start, start_theta)
    Is_curvature_threshold2, Is_error_curvature_threshold2, list_length_bezier2, Is_collision2 = planner.kari(network, goal, goal_theta)
    
    min_index_list1 = planner.kari2(network, Is_curvature_threshold1, Is_error_curvature_threshold1, list_length_bezier1, Is_collision1)
    min_index_list2 = planner.kari2(network, Is_curvature_threshold2, Is_error_curvature_threshold2, list_length_bezier2, Is_collision2)
    
    
    for path_index in range(len(min_index_list1)):
        if min_index_list1[path_index] == 99 or min_index_list2[path_index] == 99:
            continue
        
        middle_path = network[path_index]
        p.N = len(middle_path)
        #経路をspline補完し，滑らかにする
        cubicX, cubicY = GenerateInitialPath.cubic_spline_by_waypoint(middle_path)

        #middle_pathからスプライン補間を経由して経路上の姿勢を復元
        theta = GenerateInitialPath.generate_initialtheta(middle_path)
        #plt.plot(theta)
        #plt.title('middle_pathに沿ったtheta',fontname="MS Gothic")
        #plt.show()

        #middle_pathからスプライン補間を経由して経路上の曲率半径を復元
        CR_middlepath = GenerateInitialPath.generate_curvature_radius(middle_path)
        #plt.plot(CR)
        #plt.title('middle_pathに沿った曲率半径',fontname="MS Gothic")
        #plt.show()


        p0 = start
        p1_x, p1_y = [], []
        max_curvature_list = []
        error_list = []
        length_list = []
        
        #ベジエ曲線のp2に経路上のwaypointを指定
        #p2における接戦ベクトルの姿勢を設定
        p2 = [cubicX[min_index_list1[path_index]], cubicY[min_index_list1[path_index]]]
        middle_theta = theta[min_index_list1[path_index]]
        
        #p1を計算
        p1 = bezier.calc_p1(p0, p2, start_theta, middle_theta)
        p1_x.append(p1[0])
        p1_y.append(p1[1])
        
        #ベジエ曲線の最小曲率半径を計算し，格納
        min_t, min_CR = bezier.minimum_curvature_radius(p0, p1, p2)
        max_curvature_list.append(1/min_CR)
        
        #ベジエ曲線の軌道を計算
        bezier_x1, bezier_y1 = bezier.generate_bezier(p0, p1, p2)
        
        #ベジエ曲線の長さを計算し，格納
        length_bezier = bezier.calc_length_path(bezier_x1, bezier_y1)
        length_list.append(length_bezier)
        
        #ベジエ曲線の軌道に沿った曲率を計算
        bezier_k = bezier.calc_curvature(p0, p1, p2)
        
        #曲率の2乗誤差を計算し格納
        error = (bezier_k[-1] - 1/CR_middlepath[min_index_list1[path_index]]) ** 2
        error_list.append(error)
        
        bezier_theta = bezier.calc_theta(p0, p1, p2)
        print(bezier_theta[-1], theta[min_index_list1[path_index]])
        #plot.test_path(cubicX, cubicY, bezier_x1, bezier_y1)
        #plt.show()
        
        p0 = goal
        p1_x, p1_y = [], []
        max_curvature_list = []
        error_list = []
        length_list = []
        
        #ベジエ曲線のp2に経路上のwaypointを指定
        #p2における接戦ベクトルの姿勢を設定
        p2 = [cubicX[min_index_list1[path_index]], cubicY[min_index_list1[path_index]]]
        goal_theta = theta[min_index_list1[path_index]]
        
        #p1を計算
        p1 = bezier.calc_p1(p0, p2, start_theta, goal_theta)
        p1_x.append(p1[0])
        p1_y.append(p1[1])
        
        #ベジエ曲線の最小曲率半径を計算し，格納
        min_t, min_CR = bezier.minimum_curvature_radius(p0, p1, p2)
        max_curvature_list.append(1/min_CR)
        
        #ベジエ曲線の軌道を計算
        bezier_x2, bezier_y2 = bezier.generate_bezier(p0, p1, p2)
        
        #ベジエ曲線の長さを計算し，格納
        length_bezier = bezier.calc_length_path(bezier_x1, bezier_y1)
        length_list.append(length_bezier)
        
        #ベジエ曲線の軌道に沿った曲率を計算
        bezier_k = bezier.calc_curvature(p0, p1, p2)
        
        #曲率の2乗誤差を計算し格納
        error = (bezier_k[-1] - 1/CR_middlepath[min_index_list1[path_index]]) ** 2
        error_list.append(error)
        
        bezier_theta = bezier.calc_theta(p0, p1, p2)
        print(bezier_theta[-1], theta[min_index_list1[path_index]])
        #plot.test_path(cubicX, cubicY, bezier_x2, bezier_y2)
        #plt.show()
        
        plot.test2_path(cubicX, cubicY, bezier_x1, bezier_y1, bezier_x2, bezier_y2)
        plt.show()




def main2():
    #csvからnetwork情報を取得
    with open("network_circle.csv") as file:
        reader = csv.reader(file, quoting=csv.QUOTE_NONNUMERIC)
        trajectory_vectors = [row for row in reader]

    #csvから得られた情報からnetworkを構成
    network = []

    for trajectory_vector in trajectory_vectors:
        trajectory_vector = np.array(trajectory_vector)
        trajectory_matrix = util.vector_to_matrix(trajectory_vector)
        x, y = trajectory_matrix[0], trajectory_matrix[1]
        path = []
        for i in range(len(x)):
            path.append([x[i], y[i]])
        network.append(path)
        
    start = [-1, 3]
    goal = [17,-4]
    
    start_theta = 0
    goal_theta = 0
    
    planner = BezierBasePathPlanning(start, goal, start_theta, goal_theta, network)
    Is_curvature_threshold1, Is_error_curvature_threshold1, list_length_bezier1, Is_collision1 = planner.kari(network, start, start_theta)
    Is_curvature_threshold2, Is_error_curvature_threshold2, list_length_bezier2, Is_collision2 = planner.kari(network, goal, goal_theta)
    
    min_index_list1 = planner.kari2(network, Is_curvature_threshold1, Is_error_curvature_threshold1, list_length_bezier1, Is_collision1)
    min_index_list2 = planner.kari2(network, Is_curvature_threshold2, Is_error_curvature_threshold2, list_length_bezier2, Is_collision2)
    
    path_index = planner.kari3(network, min_index_list1, min_index_list2, list_length_bezier1, list_length_bezier2)
    
    
    middle_path = network[path_index]
    p.N = len(middle_path)
    #経路をspline補完し，滑らかにする
    cubicX, cubicY = GenerateInitialPath.cubic_spline_by_waypoint(middle_path)

    #middle_pathからスプライン補間を経由して経路上の姿勢を復元
    theta = GenerateInitialPath.generate_initialtheta(middle_path)
    #plt.plot(theta)
    #plt.title('middle_pathに沿ったtheta',fontname="MS Gothic")
    #plt.show()

    #middle_pathからスプライン補間を経由して経路上の曲率半径を復元
    CR_middlepath = GenerateInitialPath.generate_curvature_radius(middle_path)
    #plt.plot(CR)
    #plt.title('middle_pathに沿った曲率半径',fontname="MS Gothic")
    #plt.show()


    p0 = start
    p1_x, p1_y = [], []
    max_curvature_list = []
    error_list = []
    length_list = []
    
    #ベジエ曲線のp2に経路上のwaypointを指定
    #p2における接戦ベクトルの姿勢を設定
    p2 = [cubicX[min_index_list1[path_index]], cubicY[min_index_list1[path_index]]]
    middle_theta = theta[min_index_list1[path_index]]
    
    #p1を計算
    p1 = bezier.calc_p1(p0, p2, start_theta, middle_theta)
    p1_x.append(p1[0])
    p1_y.append(p1[1])
    
    #ベジエ曲線の最小曲率半径を計算し，格納
    min_t, min_CR = bezier.minimum_curvature_radius(p0, p1, p2)
    max_curvature_list.append(1/min_CR)
    
    #ベジエ曲線の軌道を計算
    bezier_x1, bezier_y1 = bezier.generate_bezier(p0, p1, p2)
    
    #ベジエ曲線の長さを計算し，格納
    length_bezier = bezier.calc_length_path(bezier_x1, bezier_y1)
    length_list.append(length_bezier)
    
    #ベジエ曲線の軌道に沿った曲率を計算
    bezier_k = bezier.calc_curvature(p0, p1, p2)
    
    #曲率の2乗誤差を計算し格納
    error = (bezier_k[-1] - 1/CR_middlepath[min_index_list1[path_index]]) ** 2
    error_list.append(error)
    
    bezier_theta = bezier.calc_theta(p0, p1, p2)
    print(bezier_theta[-1], theta[min_index_list1[path_index]])
    #plot.test_path(cubicX, cubicY, bezier_x1, bezier_y1)
    #plt.show()
    
    p0 = goal
    p1_x, p1_y = [], []
    max_curvature_list = []
    error_list = []
    length_list = []
    
    #ベジエ曲線のp2に経路上のwaypointを指定
    #p2における接戦ベクトルの姿勢を設定
    p2 = [cubicX[min_index_list1[path_index]], cubicY[min_index_list1[path_index]]]
    goal_theta = theta[min_index_list1[path_index]]
    
    #p1を計算
    p1 = bezier.calc_p1(p0, p2, start_theta, goal_theta)
    p1_x.append(p1[0])
    p1_y.append(p1[1])
    
    #ベジエ曲線の最小曲率半径を計算し，格納
    min_t, min_CR = bezier.minimum_curvature_radius(p0, p1, p2)
    max_curvature_list.append(1/min_CR)
    
    #ベジエ曲線の軌道を計算
    bezier_x2, bezier_y2 = bezier.generate_bezier(p0, p1, p2)
    
    #ベジエ曲線の長さを計算し，格納
    length_bezier = bezier.calc_length_path(bezier_x1, bezier_y1)
    length_list.append(length_bezier)
    
    #ベジエ曲線の軌道に沿った曲率を計算
    bezier_k = bezier.calc_curvature(p0, p1, p2)
    
    #曲率の2乗誤差を計算し格納
    error = (bezier_k[-1] - 1/CR_middlepath[min_index_list1[path_index]]) ** 2
    error_list.append(error)
    
    bezier_theta = bezier.calc_theta(p0, p1, p2)
    print(bezier_theta[-1], theta[min_index_list1[path_index]])
    #plot.test_path(cubicX, cubicY, bezier_x2, bezier_y2)
    #plt.show()
    
    plot.test2_path(cubicX, cubicY, bezier_x1, bezier_y1, bezier_x2, bezier_y2)
    plt.show()

main()