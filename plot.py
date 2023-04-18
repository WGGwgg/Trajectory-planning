import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from celluloid import Camera
from B_splines import BaseFunction,U_quasi_uniform,U_piecewise_B_Spline
from matplotlib import cm
from matplotlib.colors import LightSource
import json


def output_json(dict):
    try:
        with open("output_trajectory.json", "w", encoding='utf-8') as f:  ## 设置'utf-8'编码
            f.write(json.dumps(dict, ensure_ascii=False, indent=4))
            ## 如果ensure_ascii=True则会输出中文的ascii码，这里设为False
    except IOError:
        print("没有找到文件或读取文件失败！")
    else:
        print("内容写入文件成功！")

def smooth(pathdata, k=3, flag=2):
    # print(pathdata)
    P = np.array(pathdata).T
    # print(P)
    n = len(P) - 1  # 控制点个数-1
    ## 生成B样条曲线
    path = []  # 路径点数据存储
    Bik_u = np.zeros((n + 1, 1))
    if flag == 1:  # 均匀B样条很简单
        NodeVector = np.array([np.linspace(0, 1, n + k + 1)]
                              )  # 均匀B样条节点向量，首末值定义为 0 和 1
        # for u in np.arange(0,1,0.001):
        # u的范围为[u_{k-1},u_{n+2}],这样才是open的曲线，不然你可以使用[0,1]试试。
        for u in np.arange((k - 1) / (n + k + 1), (n + 2) / (n + k + 1), 0.005):
            for i in range(n + 1):
                Bik_u[i, 0] = BaseFunction(i, k, u, NodeVector)
            p_u = P.T @ Bik_u
            path.append(p_u.flatten())
    elif flag == 2:  # 准均匀
        NodeVector = U_quasi_uniform(n, k)
        for u in np.arange(0, 1, 0.005):
            for i in range(n + 1):
                Bik_u[i, 0] = BaseFunction(i, k, u, NodeVector)
            p_u = P.T @ Bik_u
            path.append(p_u.flatten())
    elif flag == 3:  # 分段
        NodeVector = U_piecewise_B_Spline(n, k)
        for u in np.arange(0, 1, 0.005):
            for i in range(n + 1):
                Bik_u[i, 0] = BaseFunction(i, k, u, NodeVector)
            p_u = P.T @ Bik_u
            path.append(p_u.flatten())
    path = np.array(path)
    return path

def draw_path(MapData, space_boundary, ptsData, pathsData, targData, obstacles=None, title='3D Path Plan'):
    # colors = ['b', 'c', 'm', 'y', 'b', 'w']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']*20
    fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
    # 设置x轴坐标
    x_array = np.zeros((space_boundary[0], space_boundary[1]))
    def xaxis(a, b):
        for i in range(a, b):
            x_array[i, :] = i
        return x_array
    x = xaxis(0, space_boundary[0])
    # 设置y轴坐标
    y_array = np.zeros((space_boundary[0], space_boundary[1]))
    def yaxis(a, b):
        for i in range(a, b):
            y_array[:, i] = i
        return y_array
    y = yaxis(0, space_boundary[1])

    ls = LightSource(azdeg=360, altdeg=30)
    rgb = ls.shade(MapData, cmap=cm.get_cmap('gist_earth'), vert_exag=0.1, blend_mode='soft')
    ax.plot_surface(x, y, MapData, rstride=1, cstride=1, facecolors=rgb, alpha=0.2,
                           linewidth=0, antialiased=False, shade=False)
    ax.set_xlim3d([0.0, space_boundary[0]])
    ax.set_xlabel('X')
    ax.set_ylim3d([0.0, space_boundary[1]])
    ax.set_ylabel('Y')
    ax.set_zlim3d([0.0, space_boundary[2]])
    ax.set_zlabel('Z')
    ax.set_title(title)
    # ax.scatter(ptsData[0], ptsData[1], ptsData[2], c= 'g', marker = 'o', label='Source')
    # ax.scatter(targData[0], targData[1], targData[2], c= 'r', marker = 'x', label='Dest')
    # print(ptsData,targData)
    for start,target in zip(ptsData,targData):
        ax.scatter(start[0], start[1], start[2], c= 'g', marker = 'o', label='Source')
        ax.scatter(target[0], target[1], target[2], c= 'r', marker = 'x', label='Dest')
    # print(pathsData)
    i = 0
    out_trajectory = {}
    for key in pathsData:
        # print(key)
        pathData = pathsData[key]
        # print(pathData)
        trajectory = smooth(pathData,k=3,flag=2)
        # trajectory = pathData
        # plt.plot(pathData[0], pathData[1], pathData[2], colors[i],label=key)
        # print(key,':',trajectory)
        # trajectory_temp = []
        # for it in range(len(trajectory[0])):
        #     x = float(trajectory[0][it])
        #     y = float(trajectory[1][it])
        #     z = float(trajectory[2][it])
        #     trajectory_temp.append([x,y,z])
        out_trajectory[key] = trajectory.tolist()
        plt.plot(pathData[0], pathData[1], pathData[2], colors[i], linewidth=0.8, linestyle='--', label= key)
        for ii in range(len(trajectory)):
            plt.plot(trajectory[0:ii, 0], trajectory[0:ii, 1],trajectory[0:ii, 2], colors[i])  # 路径点
        i += 1
    # ax.legend(loc = 0)
    # print(out_trajectory)
    output_json(out_trajectory)
    plt.show()

if __name__ == '__main__':
    ## 数据定义
    k = 5  # k阶、k-1次B样条
    flag = 2  # 1,2,3分别绘制均匀B样条曲线、准均匀B样条曲线,分段B样条
    # 控制点
    # ppp = np.array([
    #     [9.036145, 51.779661,1],
    #     [21.084337, 70.084746,1],
    #     [37.607573, 50.254237,1],
    #     [51.893287, 69.745763,1],
    #     [61.187608, 49.576271,1]
    # ])
    ppp = np.array([
        [
            2236.0,
            659.0,
            74.0
        ],
        [
            2162.0,
            629.0,
            151.0
        ],
        [
            2020.0,
            619.0,
            198.0
        ],
        [
            1890.0,
            649.0,
            190.0
        ],
        [
            1726.0,
            620.0,
            197.0
        ],
        [
            1551.0,
            573.0,
            132.0
        ],
        [
            1428.0,
            570.0,
            127.0
        ],
        [
            1260.0,
            605.0,
            162.0
        ],
        [
            1111.0,
            602.0,
            148.0
        ],
        [
            1014.0,
            576.0,
            142.0
        ],
        [
            840.0,
            582.0,
            139.0
        ],
        [
            820.0,
            654.0,
            149.0
        ],
        [
            708.0,
            764.0,
            140.0
        ],
        [
            533.0,
            781.0,
            131.0
        ],
        [
            452.0,
            841.0,
            80.0
        ],
        [
            286.0,
            918.0,
            64.0
        ],
        [
            236.0,
            1009.0,
            24.0
        ]])
    path = smooth(ppp,k=k,flag=flag)
    print(path)

    ## 画图
    fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
    # plt.ylim(-4, 4)
    # plt.axis([-10, 100, -15, 15])
    camera = Camera(fig)

    for i in range(len(path)):
        # plt.cla()

        plt.plot(ppp[:, 0], ppp[:, 1], ppp[:, 2], 'ro')
        plt.plot(ppp[:, 0], ppp[:, 1], ppp[:, 2], 'y')
        # 设置坐标轴显示范围
        # plt.axis('equal')
        plt.gca().set_aspect('equal')
        # 绘制路径

        plt.plot(path[0:i, 0], path[0:i, 1],path[0:i, 2], 'g')  # 路径点
        # plt.pause(0.001)
    #     camera.snap()
    # animation = camera.animate()
    # animation.save('trajectory.gif')
    plt.show()