class Topics:

    def __init__(self):
        self.camera_left_front = '/autodrive/camera_left_front/camera'
        self.camera_left_side = '/autodrive/camera_left_side/camera'
        self.camera_right_front = '/autodrive/camera_right_front/camera'
        self.camera_right_side = '/autodrive/camera_right_side/camera'
        self.camera_ir = '/autodrive/camera_ir/camera'
        self.lidar_left = '/autodrive/left_velodyne_pointcloud'
        self.lidar_right = '/autodrive/right_velodyne_pointcloud'
        self.xsens_imu = '/autodrive/xsens/imu'
        self.xsens_mag = '/autodrive/xsens/mag'
        self.xsens_gnss = '/autodrive/xsens/gnss'
        self.xsens_dela_q = '/autodrive/xsens/d_quat'
        self.xsens_pressure = '/autodrive/xsens/pressure'
        self.xsens_temp = '/autodrive/xsens/temp'
        self.xsens_time = '/autodrive/xsens/time'
        self.gnss_pose = '/autodrive/gnss_position'
        self.gnss_time = '/autodrive/gnss_time'