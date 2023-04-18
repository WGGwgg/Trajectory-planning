import numpy as np
import json
import pandas as pd
from prm import MAP_PRM
from astar import Astar
from plot import draw_path


def read_map(filepath):
    data = pd.read_csv(filepath, engine='python', encoding='utf-8', header=None)
    x_max = data.shape[0]  # 高程图的X范围
    y_max = data.shape[1]  # 高程图的Y范围
    # self.z_max = max(data.max()) + 100
    z_max = 500
    # data_np = data.values
    return data.values,[x_max,y_max,z_max]  # 矩阵形式

if __name__ == "__main__":
# 输入UE坐标
#     xmin, ymin, zmin = -22358, -11089, -6735        # UE中参考点
#     input_sss = np.array([[0, -4500, -6000], [23000, -2000, -6000],[-17000,-3000,-6000]])
#     input_ggg = np.array([[-20000, -1000, -6000], [0, 7500, -6000],[22000,4000,-6000]])
# 坐标转换
#     starts = np.ceil((input_sss - [xmin,ymin,zmin])/10)
#     goals = np.ceil((input_ggg - [xmin,ymin,zmin])/10)

# test_dem.csv中给定起点终点
    starts = np.array([[60,10,74],[200,10,74],[255,10,74],[10,115,74],[350,25,74]])
    goals = np.array([[220,180,74],[150,210,74],[200,200,74],[300,180,74],[330,150,74]])

    # 构建无向图
# 读取高程图
    dem_data,space_boundary = read_map('test_dem.csv')
# PRM生成
    map_real = MAP_PRM(data=dem_data, smaple_num=5000, radius=50, space_boundary=space_boundary, safe_R=5, input_starts=starts, input_goals=goals)
    map_real.PRM_main()
    vertex = map_real.vertex
    edge = map_real.edges

# 或导入无向图G=(V,E)，其中vertex集合中按顺序存放所有点的坐标，E中按照vertex_id：(vertex_id,distance)格式
#     with open("output_vertex.json", 'r', encoding='utf-8') as f:
#         vertex = np.array(json.loads(f.read()))
#     with open("output_edge.json", 'r', encoding='utf-8') as f:
#         edge = json.loads(f.read())

    # 参数设置
    star_point = starts
    target_point = goals
    seed = np.random.randint(0, 99999)      # 随机数种子
    agents_num = len(star_point)            # 飞行器数量

    # use_astar = None                    # A_star算法
    use_astar = True

    if use_astar is None:
        print("Error: You need to select a algorithm to plan path: Astar")
    else:
        print('Space: ', space_boundary)
        print('Agents number: ', agents_num)
        print('Seed: ', seed)
        print('Starts:', star_point)
        print('Goals:', target_point)
        # 使用A_star算法
        if use_astar:
            print('Algorithm: Astar')
            pathsData = {}                                  # 存放所有飞行器路径{[点集合],[],[]}
            for it in range(agents_num):
                # 配置算法，输入所有飞行器起点、终点、障碍物、空间边界、运动方式
                astar = Astar(vertex,edge,star_point[it],target_point[it])
                pathData = astar.search()                   # 算法搜索得到路径
                if (pathData[:, -1] == pathData[:, -2]).all():
                    pathData = pathData[:, :-1]
                pathsData[f"Path{it} With Obstacle"] = pathData     # 记录所有飞行器路径
                print("Reached!")
            print(pathsData)
            # 绘制所有飞行器路径，输入空间边界、起点、路径、终点、障碍物
            draw_path(dem_data,space_boundary=space_boundary,ptsData=star_point,pathsData= pathsData,
                  targData=target_point ,title = "Path Plan with A*")
    print("Finish!")