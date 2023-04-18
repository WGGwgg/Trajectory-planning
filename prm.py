import json
import math
import numpy as np
import matplotlib.pyplot as plt

class MAP_PRM():
    def __init__(self,data,smaple_num,radius,space_boundary,safe_R,input_starts,input_goals):
        self.sample_num = smaple_num
        self.radius = radius
        self.safe_R = safe_R
        self.input_s = input_starts
        self.input_g = input_goals
        self.MapData = data
        self.space_boundary = space_boundary
        self.x_max, self.y_max, self.z_max = self.space_boundary

    def feasible(self,p):
        # 可以加入与空间中障碍物的安全距离
        return True if p[2] > np.max(self.MapData[(p[0]-self.safe_R):(p[0]+self.safe_R),(p[1]-self.safe_R):(p[1]+self.safe_R)])+self.safe_R else False

    def check_path(self,p,newp):
        check = True
        disr = math.atan2(newp[0]-p[0],newp[1]-p[1])
        for r in np.arange(1,np.sqrt((newp[0]-p[0])**2+(newp[1]-p[1])**2),int(self.safe_R/2)):
            dp = [r*math.sin(disr), r*math.cos(disr), r*(newp[2] - p[2]) / math.sqrt((newp[0] - p[0])** 2 + (newp[1] - p[1])** 2)]
            posCheck = list(map(lambda x, y: x + y, p, dp))
            # posCheck = np.floor(posCheck)
            if posCheck[2] < np.max(self.MapData[int(posCheck[0])-self.safe_R:int(posCheck[0])+self.safe_R,int(posCheck[1])-self.safe_R:int(posCheck[1])+self.safe_R])+self.safe_R:
                check = False
                break
        return check

    def PRM_main(self):
        # 产生随机点
        self.vertex = np.concatenate((self.input_s,self.input_g),axis=0)
        # print(self.vertex)
        space_boundary = [self.x_max, self.y_max, self.z_max]
        fig,ax = plt.subplots(subplot_kw=dict(projection='3d'))
        ax.set_xlim3d([0.0, space_boundary[0]])
        ax.set_xlabel('X')
        ax.set_ylim3d([0.0, space_boundary[1]])
        ax.set_ylabel('Y')
        ax.set_zlim3d([0.0, space_boundary[2]])
        ax.set_zlabel('Z')
        ax.set_title(f'Sample num = {self.sample_num}')
        while len(self.vertex)< self.sample_num:
            sample_point = (np.random.randint(self.safe_R,self.x_max-self.safe_R), np.random.randint(self.safe_R,self.y_max-self.safe_R), np.random.randint(self.safe_R,self.z_max-self.safe_R))
            # sample_point = (np.random.randint(1, self.x_max),
            #                 np.random.randint(1, self.y_max),
            #                 np.random.randint(1, self.z_max))
            if self.feasible(sample_point):
                self.vertex= np.append(self.vertex,[sample_point],axis=0)
                # ax.scatter(sample_point[0],sample_point[1],sample_point[2],color='b',s=2**2,alpha=0.4)
        # 建立无向图
        self.edges = {str(i):[] for i in range(self.sample_num)}
        for i in range(self.sample_num):
            print(f'Vertex connect processing: {i+1}/{self.sample_num}')
            for j in range(i+1,self.sample_num):
                # 计算两点间的空间距离
                distance = math.sqrt((self.vertex[i][0] - self.vertex[j][0]) ** 2 + (self.vertex[i][1] - self.vertex[j][1]) ** 2 + (self.vertex[i][2] - self.vertex[j][2]) ** 2)
                if distance < self.radius:
                    if self.check_path(self.vertex[i],self.vertex[j]):
                        ax.plot([self.vertex[i][0],self.vertex[j][0]],[self.vertex[i][1],self.vertex[j][1]],[self.vertex[i][2],self.vertex[j][2]], alpha=0.4,color='g',linewidth=0.2)
                        self.edges[str(i)].append([int(j),distance])
                        self.edges[str(j)].append([int(i),distance])

        # print(self.vertex)
        # print(self.edges)
        with open('output_vertex.json',encoding='utf-8',mode='w') as f:
            f.write(json.dumps(self.vertex.tolist(), ensure_ascii=False, indent=4))
        with open('output_edge.json',encoding='utf-8',mode='w') as f:
            f.write(json.dumps(self.edges, ensure_ascii=False, indent=4))
        plt.show()
if __name__ == '__main__':
    starts = np.array(
        [[50, 8, 100], [250, 8, 100], [175, 8, 100], [330, 40, 100], [10, 50, 100], [350, 25, 100], [350, 75, 100],
         [350, 125, 100], [350, 175, 100], [350, 200, 100]])
    goals = np.array(
        [[225, 188, 100], [200, 200, 100], [150, 220, 100], [125, 150, 100], [300, 190, 100], [180, 180, 100],
         [125, 175, 100], [125, 210, 100], [150, 100, 100], [125, 15, 100]])
    map_real = MAP_PRM('test_dem.csv', smaple_num=1000, radius=40, safe_R=5,input_starts=starts,input_goals=goals)
    map_real.PRM_main()