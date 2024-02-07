#初期パスを生成するファイル

import numpy as np
from scipy import interpolate
from param import Parameter as p

########
#WayPointから3次スプライン関数を生成し、状態量をサンプリングする
########

#3次スプライン関数の生成
def cubic_spline():   
    x, y = [], []
    for i in range(len(p.WayPoint)):
        x.append(p.WayPoint[i][0])
        y.append(p.WayPoint[i][1])
        
    tck,u = interpolate.splprep([x,y], k=3, s=0) 
    u = np.linspace(0, 1, num=p.N, endpoint=True)
    spline = interpolate.splev(u, tck)
    cubicX = spline[0]
    cubicY = spline[1]
    return cubicX, cubicY

#直線補間の生成
def interp_1d():
    x1, y1 = p.initial_x, p.initial_y
    x2, y2 = p.terminal_x, p.terminal_y
    
    x_list = []
    y_list = []
    
    for i in range(p.N):
        x = x1 + (x2 - x1) * i / (p.N - 1)
        y = y1 + (y2 - y1) * i / (p.N - 1)
        
        x_list.append(x)
        y_list.append(y)
        
    return x_list, y_list
    

#3次スプライン関数の生成(経路が関数の引数として与えられる場合)
def cubic_spline_by_waypoint(waypoint):   
    x, y = [], []
    for i in range(len(waypoint)):
        x.append(waypoint[i][0])
        y.append(waypoint[i][1])
        
    tck,u = interpolate.splprep([x,y], k=3, s=0) 
    u = np.linspace(0, 1, num=p.N, endpoint=True)
    spline = interpolate.splev(u, tck)
    cubicX = spline[0]
    cubicY = spline[1]
    return cubicX, cubicY

#x, yからΘとφを生成する
def generate_initialpath(cubicX, cubicY):
    #nd.arrayに変換
    x = np.array(cubicX)
    y = np.array(cubicY)
    
    #x, yの差分を計算
    deltax = np.diff(x)
    deltay = np.diff(y)
    
    #x, y の差分からthetaを計算
    #theta[0]を初期値に置き換え、配列の最後に終端状態を追加
    theta = np.arctan(deltay / deltax)
    theta[0] = p.initial_theta
    theta = np.append(theta, p.terminal_theta)
    
    #thetaの差分からphiを計算
    #phi[0]を初期値に置き換え配列の最後に終端状態を追加
    deltatheta = np.diff(theta)
    phi = deltatheta / p.dt
    phi[0] = p.initial_phi
    phi = np.append(phi, p.terminal_phi)
    
    #x,yの差分からvを計算
    #phi[0]を初期値に置き換え配列の最後に終端状態を追加
    v = np.sqrt((deltax ** 2 + deltay ** 2) / p.dt)
    v[0] = p.initial_v
    v = np.append(v, p.terminal_v)
    return x, y, theta, phi, v


#waypointからΘを生成する
def generate_initialtheta(waypoint):
    x, y = [], []
    for i in range(len(waypoint)):
        x.append(waypoint[i][0])
        y.append(waypoint[i][1])
        
    tck,u = interpolate.splprep([x,y], k=3, s=0) 
    # B-Spline曲線を評価
    x_eval, y_eval = interpolate.splev(np.linspace(0, 1, p.N), tck)

    # 1階微分を生成
    dx_dt, dy_dt = interpolate.splev(np.linspace(0, 1, p.N), tck, der=1)
    
    dx_dt = np.array(dx_dt)
    dy_dt = np.array(dy_dt)
    
    theta = np.arctan(dy_dt/dx_dt)
    return theta


#waypointからcurvature_radiusを生成する
def generate_curvature_radius(waypoint):
    x, y = [], []
    for i in range(len(waypoint)):
        x.append(waypoint[i][0])
        y.append(waypoint[i][1])
        
    tck,u = interpolate.splprep([x,y], k=3, s=0) 
    # B-Spline曲線を評価
    x_eval, y_eval = interpolate.splev(np.linspace(0, 1, p.N), tck)

    # 1階微分を生成
    dx_dt, dy_dt = interpolate.splev(np.linspace(0, 1, p.N), tck, der=1)
    
    # 2階微分を生成
    d2x_dt2, d2y_dt2 = interpolate.splev(np.linspace(0, 1, p.N), tck, der=2)
    
    dx_dt = np.array(dx_dt)
    dy_dt = np.array(dy_dt)
    
    d2x_dt2 = np.array(d2x_dt2)
    d2y_dt2 = np.array(d2y_dt2)
    
    CR = ((dx_dt)**2 + (dy_dt)**2)**(1.5) / np.abs(dx_dt*d2y_dt2 - dy_dt*d2x_dt2)
    
    return CR
