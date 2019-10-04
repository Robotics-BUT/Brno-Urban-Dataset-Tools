import subprocess
from utils import constants

class VideoToImagesConverter:

    def convert_recording(self, source_folder, destination_folder):
        cameras = constants.Cameras()
        for camera in cameras.as_list():
            self.convert_single_camera(source_folder+camera+'/', destination_folder+camera+'/')

    def convert_single_camera(self, source_folder, destination_folder):
        bashCmd = 'mkdir ' + destination_folder
        process = subprocess.Popen(bashCmd.split(), stdout=subprocess.PIPE)
        process.wait()

        bashCmd = 'ffmpeg -i ' + source_folder + 'video.mp4 -qscale:v 2 -start_number 0 ' + destination_folder + '/frame%06d.jpeg'
        process = subprocess.Popen(bashCmd.split(), stdout=subprocess.PIPE)
        process.wait()