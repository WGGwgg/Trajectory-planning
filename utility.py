import numpy as np

#def check_in_area(arr, x_limit, y_limit, z_limit):
#    if arr[0] >= x_limit[0] and arr[0] <= x_limit[1]:
#        if arr[1] >= y_limit[0] and arr[1] <= y_limit[1]:
#            if arr[2] >= z_limit[0] and arr[2] <= z_limit[1]:
#                return True
#    return False

# 检测位置是否在边界内
def check_in_area(loc, space_boundary):
    if loc[0] >= 0 and loc[0] < space_boundary[0]:
        if loc[1] >= 0 and loc[1] < space_boundary[1]:
            if loc[2] >= 0 and loc[2] < space_boundary[2]:
                return True
    return False

# 不太懂这个函数的功能
def check_in_area_with_R(arr1, arr2, r):
    if np.max(np.abs(arr1 - arr2)) <= r:
        return True
    return False

# 检测飞行器是否在某个点
def is_exist_same_loc(loc, agents_pos):
    if agents_pos is None:
        return False
    idxes_match = (agents_pos[0, :] == loc[0])&(agents_pos[1, :] == loc[1])&\
                    (agents_pos[2, :] == loc[2])

    pos_in_area = np.where(idxes_match == True)[0]
    # print(pos_in_area)
    if len(pos_in_area)>0:
    # pos_in_area = np.where(idxes_match == True)
    # if all(pos_in_area):
        return True
    return False

# 不太懂这个函数的功能
def remove_same_loc(loc, agents_pos):
    if agents_pos is not None:
        idxes_match = (agents_pos[0, :] == loc[0])&(agents_pos[1, :] == loc[1])&\
                        (agents_pos[2, :] == loc[2])

        pos_in_area = np.where(idxes_match == True)[0]
        agents_pos = np.delete(agents_pos, pos_in_area, 1)
    return agents_pos

# 计算两点之间的Manhattan距离
def cal_Manhattan_dist(source, dest, unit_c = 1):
    # print(source)
    # print(dest)
    dist = np.sum(np.abs(dest - source))
    return unit_c*dist

# 计算两点之间的Euclidean距离
def cal_Euclidean_dist(source, dest, unit_c = 1):
    dist = np.linalg.norm(dest - source)
    return unit_c*dist