from video_to_images_converter import VideoToImagesConverter

def main():

    source_folder = 'SOURCE_PATH/'
    destination_folder = 'DESTINATION_PATH/'

    converter = VideoToImagesConverter()
    #converter.convert_single_camera(source_folder, destination_folder)
    converter.convert_recording(source_folder, destination_folder)

if __name__ == "__main__":
    main()