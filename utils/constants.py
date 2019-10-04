

class Cameras:

    def __init__(self):
        self.camera_left_front = 'camera_left_front'
        self.camera_left_side = 'camera_left_side'
        self.camera_right_front = 'camera_right_front'
        self.camera_righ_side = 'camera_right_side'
        self.camera_ir = 'camera_ir'

    def as_list(self):
        return [
            self.camera_left_front,
            self.camera_left_side,
            self.camera_right_front,
            self.camera_righ_side,
            self.camera_ir
        ]


class Lidars:

    def __init__(self):
        self.lidar_left = 'lidar_left'
        self.lidar_right = 'lidar_right'

    def as_list(self):
        return [
            self.lidar_left,
            self.lidar_right
        ]

