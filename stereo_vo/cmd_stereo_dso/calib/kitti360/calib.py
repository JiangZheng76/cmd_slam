import cv2
import numpy as np

# 定义投影矩阵
P = np.array([
    [552.554261, 0.000000, 682.049453, -328.318735],
    [0.000000, 552.554261, 238.769549, 0.000000],
    [0.000000, 0.000000, 1.000000, 0.000000]
])

def decompose_projection_matrix(proj_matrix):
    K, R, T, _, _, _, _ = cv2.decomposeProjectionMatrix(proj_matrix)
    T = T[:3]  # 取前三个元素作为平移向量

    return K, R, T

K, R, T = decompose_projection_matrix(P)
print("K:\n",K)
print("R:\n",R)
print("T:\n",T)