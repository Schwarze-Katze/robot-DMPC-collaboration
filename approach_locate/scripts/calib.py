import cv2
import numpy as np
import glob

# 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

# 获取标定板角点的位置
objp = np.zeros((8 * 13, 3), np.float32)
# 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
objp[:, :2] = np.mgrid[0:13, 0:8].T.reshape(-1, 2)

obj_points = []  # 存储3D点
img_points = []  # 存储2D点

images = glob.glob("/mnt/c/Users/katze/Documents/MATLAB/imagecap/*.jpg")
for fname in images:
    img = cv2.imread(fname)

    cv2.imshow('img', img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    # gray = cv2.resize(gray, None, fx=0.5, fy=0.5,
    #                   interpolation=cv2.INTER_LINEAR)
    # img = cv2.resize(img, None, fx=0.5, fy=0.5,
    #                   interpolation=cv2.INTER_LINEAR)

    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(
        gray, (13, 8), cv2.CALIB_CB_NORMALIZE_IMAGE)

    print(ret)

    if ret:

        obj_points.append(objp)

        corners2 = cv2.cornerSubPix(
            gray, corners, (15, 15), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
        # print(corners2)
        if [corners2]:
            img_points.append(corners2)
        else:
            img_points.append(corners)

        cv2.drawChessboardCorners(
            img, (13, 8), corners, ret)  # OpenCV的绘制函数一般无返回值

        cv2.imshow('img', img)
        cv2.waitKey(2000)

print(len(img_points))
cv2.destroyAllWindows()

# 标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, size, None, None)

# 创建一个 4x4 的单位矩阵
mtx_4 = np.eye(4)

# 将 3x3 的 mtx 嵌入到 4x4 矩阵的左上角
mtx_4[:3, :3] = mtx

print("ret:", ret)
print("mtx:\n", mtx_4.flatten())  # 内参数矩阵
print("dist:\n", dist)  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
print("rvecs:\n", rvecs)  # 旋转向量  # 外参数
print("tvecs:\n", tvecs)  # 平移向量  # 外参数

print("-----------------------------------------------------")
