import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from param import Parameter as p

#2次ベジエ曲線の描画テスト
#bezier曲線を作成
def generate_bezier(p0, p1, p2):
    x0, x1, x2 = p0[0], p1[0], p2[0]
    y0, y1, y2 = p0[1], p1[1], p2[1]
    
    t = np.linspace(0, 1, 100)
    
    x = (1 - t)**2*x0 + 2*(1 - t)*t*x1 + t**2*x2
    y = (1 - t)**2*y0 + 2*(1 - t)*t*y1 + t**2*y2
    
    return x, y


#生成されるベジエ曲線の媒介変数に沿った曲率を計算
def calc_curvature(p0, p1, p2):
    x0, x1, x2 = p0[0], p1[0], p2[0]
    y0, y1, y2 = p0[1], p1[1], p2[1]
    
    t = np.linspace(0, 1, 100)
    
    #x',x",y',y"の係数を計算
    a_x = 2*(x0 - 2*x1 + x2)
    b_x = 2*(-x0 + x1)
    
    a_y = 2*(y0 - 2*y1 + y2)
    b_y = 2*(-y0 + y1)
    
    R = ((a_x**2 + a_y**2)*t**2 + 2*(a_x*b_x + a_y*b_y)*t + b_x**2 + b_y**2) ** (3/2) / np.abs(a_y*b_x - a_x*b_y)
    
    return 1/R

#曲線に沿ったベジエ曲線の角度(姿勢)を計算
def calc_theta(p0, p1, p2):
    x0, x1, x2 = p0[0], p1[0], p2[0]
    y0, y1, y2 = p0[1], p1[1], p2[1]
    
    t = np.linspace(0, 1, 100)
    
    #x',x",y',y"の係数を計算
    a_x = 2*(x0 - 2*x1 + x2)
    b_x = 2*(-x0 + x1)
    
    a_y = 2*(y0 - 2*y1 + y2)
    b_y = 2*(-y0 + y1)
    
    xdot = a_x*t + b_x
    ydot = a_y*t + b_y
    
    theta = np.arctan(ydot / xdot)
    for i in range(len(theta)):
        if xdot[i] < 0 and ydot[i] > 0:
            theta[i] += np.pi
        elif xdot[i] < 0 and ydot[i] < 0:
            theta[i] -= np.pi
            
    return theta

#作成したベジエ曲線の曲率半径の最小値とその時の媒介変数の値を返す
def minimum_curvature_radius(p0, p1, p2):
    x0, x1, x2 = p0[0], p1[0], p2[0]
    y0, y1, y2 = p0[1], p1[1], p2[1]
    
    #x',x",y',y"の係数を計算
    a_x = 2*(x0 - 2*x1 + x2)
    b_x = 2*(-x0 + x1)
    
    a_y = 2*(y0 - 2*y1 + y2)
    b_y = 2*(-y0 + y1)
    
    t_min = - (a_x*b_x + a_y*b_y) / (a_x**2 + a_y**2)
    CR_min = ((a_x**2 + a_y**2)*t_min**2 + 2*(a_x*b_x + a_y*b_y)*t_min + b_x**2 + b_y**2) ** (3/2) / np.abs(a_y*b_x - a_x*b_y)
    
    return t_min, CR_min


#2点p0,p2とその点における姿勢からp1を計算する
def calc_p1(p0, p2, theta0, theta2):
    m0, m2 = np.tan(theta0), np.tan(theta2)
    x0, x2 = p0[0], p2[0]
    y0, y2 = p0[1], p2[1]
    
    x1 = ((y2 - m2*x2) - (y0 - m0*x0)) / (m0 - m2)
    y1 = m0*x1 + (y0 - m0*x0)
    
    return [x1, y1]

#与えられた経路の長さを計算する
def calc_length_path(x, y):
    length = 0
    
    for i in range(0, len(x)-1):
        length += ((x[i] - x[i+1])**2 + (y[i] - y[i+1])**2) ** (0.5)
        
    return length


