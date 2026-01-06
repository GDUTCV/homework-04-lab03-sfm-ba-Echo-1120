import numpy as np
import cv2


def compute_ba_residuals(parameters: np.ndarray, intrinsics: np.ndarray, num_cameras: int, points2d: np.ndarray,
                         camera_idxs: np.ndarray, points3d_idxs: np.ndarray) -> np.ndarray:
    """
    For each point2d in <points2d>, find its 3d point, reproject it back into the image and return the residual
    i.e. euclidean distance between the point2d and reprojected point.

    Args:
        parameters: list of camera parameters [r1, r2, r3, t1, t2, t3, ...] where r1, r2, r3 corresponds to the
                    Rodriguez vector. There are 6C + 3M parameters where C is the number of cameras
        intrinsics: camera intrinsics 3 x 3 array
        num_cameras: number of cameras, C
        points2d: N x 2 array of 2d points
        camera_idxs: camera_idxs[i] returns the index of the camera for points2d[i]
        points3d_idxs: points3d[points3d_idxs[i]] returns the 3d point corresponding to points2d[i]

    Returns:
        N residuals

    """
    num_camera_parameters = 6 * num_cameras
    camera_parameters = parameters[:num_camera_parameters]
    points3d = parameters[num_camera_parameters:]
    num_points3d = points3d.shape[0] // 3
    points3d = points3d.reshape(num_points3d, 3)

    camera_parameters = camera_parameters.reshape(num_cameras, 6)
    camera_rvecs = camera_parameters[:, :3]
    camera_tvecs = camera_parameters[:, 3:]

    extrinsics = []
    for rvec in camera_rvecs:
        rot_mtx, _ = cv2.Rodrigues(rvec)
        extrinsics.append(rot_mtx)
    extrinsics = np.array(extrinsics)  # C x 3 x 3
    extrinsics = np.concatenate([extrinsics, camera_tvecs.reshape(-1, 3, 1)], axis=2)  # C x 3 x 4

    residuals = np.zeros(shape=points2d.shape[0], dtype=float)
    """ 
    YOUR CODE HERE: 
    NOTE: DO NOT USE LOOPS 
    HINT: I used np.matmul; np.sum; np.sqrt; np.square, np.concatenate etc.
    """
    # 1. 获取每个观测点对应的相机外参矩阵
    # camera_idxs 形状: (N,), extrinsics 形状: (C, 3, 4) -> 结果形状: (N, 3, 4)
    current_extrinsics = extrinsics[camera_idxs]

    # 2. 获取每个观测点对应的 3D 坐标
    # points3d_idxs 形状: (N,), points3d 形状: (M, 3) -> 结果形状: (N, 3)
    current_points3d = points3d[points3d_idxs]

    # 3. 将 3D 点转换为齐次坐标 (X, Y, Z, 1)
    # 形状: (N, 4)
    points3d_hom = np.concatenate([current_points3d, np.ones((current_points3d.shape[0], 1))], axis=1)

    # 4. 将点从世界坐标系变换到相机坐标系: P_cam = [R|t] * P_world
    # 批量矩阵乘法: (N, 3, 4) @ (N, 4, 1) -> (N, 3, 1)
    points_cam = np.matmul(current_extrinsics, points3d_hom.reshape(-1, 4, 1))

    # 5. 将点投影到图像平面 (像素坐标系): P_img = K * P_cam
    # 批量矩阵乘法: (3, 3) @ (N, 3, 1) -> (N, 3, 1) (NumPy 会自动广播 intrinsics)
    points_proj_hom = np.matmul(intrinsics, points_cam)
    points_proj_hom = points_proj_hom.reshape(-1, 3)

    # 6. 透视除法 (归一化): u = x/z, v = y/z
    # 形状: (N, 2)
    points_proj_2d = points_proj_hom[:, :2] / points_proj_hom[:, 2:]

    # 7. 计算残差 (欧氏距离)
    diff = points2d - points_proj_2d
    residuals = np.sqrt(np.sum(np.square(diff), axis=1))
    
    """ END YOUR CODE HERE """
    return residuals