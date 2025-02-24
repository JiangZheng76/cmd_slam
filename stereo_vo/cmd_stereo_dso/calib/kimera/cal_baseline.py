import numpy as np

def se3_matrix(rotation, translation):
    """
    构建一个 SE(3) 矩阵。

    :param rotation: 3x3 的旋转矩阵
    :param translation: 3x1 的平移向量
    :return: 4x4 的 SE(3) 矩阵
    """
    se3 = np.eye(4)  # 创建一个 4x4 的单位矩阵
    se3[:3, :3] = rotation  # 设置旋转部分
    se3[:3, 3] = translation  # 设置平移部分
    return se3

# 定义两个旋转矩阵
T_w_left = np.array([
    [0.99985945, -0.01443721,  0.00852319, -0.02647408],
    [0.01453776,  0.99982404, -0.01185494,  0.00914898],
    [-0.00835054,  0.01197718,  0.99989340,  0.02575617],
    [0.00000000,  0.00000000,  0.00000000,  1.00000000]
])

T_w_right = np.array([
    [0.99982121, -0.01427648,  0.01239860,  0.06845528],
    [0.01439878,  0.99984800, -0.00983129,  0.01070672],
    [-0.01225636,  0.01000806,  0.99987480,  0.02565437],
    [0.00000000,  0.00000000,  0.00000000,  1.00000000]
])

T_right_w = np.linalg.inv(T_w_right)

# 计算两个 SE(3) 矩阵的乘积
T_product = np.dot(T_right_w, T_w_left)

print("T1:\n", T_w_right)
print("T2:\n", T_w_left)
print("T_product:\n", T_product)