import env
from param import Parameter as p
import plot
import GenerateInitialPath
import numpy as np
import util
import matplotlib.pyplot as plt
import bezier
import csv

#スタート地点と姿勢を設定
start = [0, 0]
start_theta = np.pi/4

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

#最大曲率・曲率誤差の制約を満たすかどうかを格納するlistと閾値を用意
curvature_threshold= np.tan(p.phi_max)/p.L #最大曲率
curvature_threshold= np.tan(np.pi/4)/p.L
error_curvature_threshold = 0.5
Is_curvature_threshold = []
Is_error_curvature_threshold = []

#計算されたベジエ曲線の長さを記録するlist
list_length_bezier = []

#networkから1本ずつmiddle_pathを取り出し，全てのwaypointに対してベジエ曲線を計算する
#計算されたベジエ曲線が制約を満たすかどうかを確かめ，長さを計算する
for middle_path in network:
    p.N = len(middle_path)

    #経路をspline補完し，滑らかにする
    cubicX, cubicY = GenerateInitialPath.cubic_spline_by_waypoint(middle_path)
    #theta, phi, v = np.array([0]*p.N), np.array([0]*p.N), np.array([0]*p.N)
    #trajectory_matrix = np.array([cubicX, cubicY, theta, phi, v])
    #trajectory_vector = util.matrix_to_vector(trajectory_matrix)
    #plot.vis_path(trajectory_vector)

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
    for i in range(len(theta)):
        #ベジエ曲線のp2に経路上のwaypointを指定
        #p2における接戦ベクトルの姿勢を設定
        p2 = [cubicX[i], cubicY[i]]
        middle_theta = theta[i]
        
        #p1を計算
        p1 = bezier.calc_p1(p0, p2, start_theta, middle_theta)
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
        
        #bezier_theta = bezier.calc_theta(p0, p1, p2)
        #print(bezier_theta[-1], theta[i])
        #plot.test_path(cubicX, cubicY, bezier_x, bezier_y)
        #plt.show()

    #ベジエ曲線が制約を満たすかどうかを確認するlist
    check_curvature_threshold = []
    check_error_curvature_threshold = []

    #curvatureが閾値より小さければtrue,そうでなければfalse
    for curvature in max_curvature_list:
        if curvature < curvature_threshold:
            check_curvature_threshold.append('True')
        else:
            check_curvature_threshold.append('False')

    #print(check_curvature_threshold)
    #plt.plot(max_curvature_list)
    #plt.title('ベジエ曲線の最小曲率半径',fontname="MS Gothic")
    #plt.show()

    #errorが閾値より地位避けれがtrue,そうでなければfalse
    for error in error_list:
        if error < error_curvature_threshold:
            check_error_curvature_threshold.append('True')
        else:
            check_error_curvature_threshold.append('False')
            
    #print(check_error_curvature_threshold)
    #plt.plot(error_list)
    #plt.title('ベジエ曲線とmiddle_pathの接続における曲率の二乗誤差',fontname="MS Gothic")
    #plt.show()
    
    #check_listをappendする
    Is_curvature_threshold.append(check_curvature_threshold)
    Is_error_curvature_threshold.append(check_error_curvature_threshold)
    
    #距離のlistをappendする
    list_length_bezier.append(length_list)
    
    
print(Is_curvature_threshold)
print(Is_curvature_threshold)
print(list_length_bezier)


min_index_list = []
for i in range(len(network)):
    min_length = 999999
    min_index = 99
    for j in range(len(network[i])):
        if Is_curvature_threshold[i][j] == 'True' and Is_error_curvature_threshold[i][j] == 'True':
            if list_length_bezier[i][j] <= min_length:
                min_length = list_length_bezier[i][j]
                min_index = j
            else:
                pass
        else:
            pass
    min_index_list.append(min_index)
    print(min_length)
print(min_index_list)



for path_index in range(len(min_index_list)):
    if min_index_list[path_index] == 99:
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
    p2 = [cubicX[min_index_list[path_index]], cubicY[min_index_list[path_index]]]
    goal_theta = theta[min_index_list[path_index]]
    
    #p1を計算
    p1 = bezier.calc_p1(p0, p2, start_theta, goal_theta)
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
    error = (bezier_k[-1] - 1/CR_middlepath[min_index_list[path_index]]) ** 2
    error_list.append(error)
    
    bezier_theta = bezier.calc_theta(p0, p1, p2)
    print(bezier_theta[-1], theta[min_index_list[path_index]])
    plot.test_path(cubicX, cubicY, bezier_x, bezier_y)
    plt.show()
