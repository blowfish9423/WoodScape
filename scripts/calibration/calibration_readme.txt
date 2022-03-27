# Calibration

Each image is provided with a dedicated calibration file including intrinsic and extrinsic parameters as well as a descriptive name. 
The name is one of "FV", "MVR", "MVL" or "RV", short for "Front View", "Mirror View Right", "Mirror View Left", "Rear View". 

#   每一副图像配有专用校准文件，包括内在和外在参数以及描述性名称。
    FV-->前视
    RV-->后视
    MVL-->左视
    MVR-->右视
## Extrinsic

The vehicle coordinate system, which follows the ISO 8855 convention, is anchored to the ground below the midpoint of the rear axle. The X axis points in driving direction, the Y axis points to the left side of the vehicle and the Z axis points up from the ground. 
The camera sensor's coordinate system is based on OpenCV. The X axis points to the right along the horizontal sensor axis, the Y axis points downwards along the vertical sensor axis and the Z axis points in viewing direction along the optical axis to maintain the right-handed system. 
The values of the translation are given in meters and the rotation is given as a quaternion. They describe the coordinate transformation from the camera coordinate system to the vehicle coordinate system. The rotation matrix can be obtained using e.g. `scipy.spatial.transform.Rotation.from_quat`.
    车辆坐标系，遵循ISO 8855公约，固定在后轴中点下方的地面上。 X 轴指向行驶方向，Y 轴指向车辆左侧，Z 轴指向地面上方。
    相机传感器坐标系统基于openCV, X 轴沿传感器水平轴指向右侧，Y 轴沿传感器垂直轴指向下方，Z 轴沿光轴指向观察方向，以保持右手系。
    平移的值以米为单位，旋转以四元数的形式给出。 它们描述了从相机坐标系到车辆坐标系的坐标变换。 可以使用例如获得旋转矩阵 `scipy.spatial.transform.Rotation.from_quat`。

## Intrinsic

The intrinsic calibration is given in a calibration model that describes the radial distortion using a 4th order polynomial 
rho(theta) = k1 * theta + k2 * theta ** 2 + k3 * theta ** 3 + k4 * theta ** 4,
where theta is the angle of incidence with respect to the optical axis and rho is the distance between the image center and projected point. The coefficients k1, k2, k3 and k4 are given in the calibration files.
The image width and height as well as the offset (cx, cy) of the principal point are given in pixels.

内在校准在一个校准模型中给出，该模型使用四阶多项式描述径向失真:
rho(theta) = k1 * theta + k2 * theta ** 2 + k3 * theta ** 3 + k4 * theta ** 4,
其中中，theta 是相对于光轴的入射角，rho 是图像中心和投影点之间的距离。 系数 k1、k2、k3 和 k4 在校准文件中给出。
  基本点的图像宽度和高度以及偏移量（cx，cy）以像素为单位。

For completeness, the projection of a 3D point (X, Y, Z) given in camera coordinates to image coordinates (u, v) looks like:
为了完整起见，相机坐标中给定的 3D 点 (X, Y, Z) 到图像坐标 (u, v) 的投影如下所示：

https://blog.csdn.net/u010128736/article/details/52864024?utm_medium=distribute.pc_relevant.none-task-blog-baidujs-3

chi = sqrt( X ** 2 + Y ** 2)
theta = arctan2( chi, Z ) = pi / 2 - arctan2( Z, chi )
rho = rho(theta)
u’ = rho * X / chi if chi != 0 else 0
v’ = rho * Y / chi if chi != 0 else 0
u = u’ + cx + width / 2 - 0.5
v = v’ * aspect_ratio + cy + height / 2 - 0.5

The last two lines show the final conversion to image coordinates assuming that the origin of the image coordinate system is located in the center of the upper left pixel. The projection itself is implemented in `projection.py` which can be found at https://github.com/valeoai/WoodScape/tree/master/scripts/calibration.
