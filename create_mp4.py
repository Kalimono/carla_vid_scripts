import cv2
import os
import re
import numpy as np
import time

def natural_sort_key(s):
    """
    Key function for natural sorting of strings.
    """
    return [int(text) if text.isdigit() else text.lower() for text in re.split(r'(\d+)', s)]

def pngs_to_mp4(folder_paths, output_file, fps=60):
    """
    Converts PNG images from multiple folders into an MP4 video after concatenating them horizontally
    with a 16-pixel-wide black border in between. Images in the first and last folder are inverted horizontally.
    
    Parameters:
    - folder_paths: List of folder paths containing PNG images.
    - output_file: Path for the output MP4 file.
    - fps: Frames per second for the video.
    """
    # Read and sort images from each folder
    image_lists = []
    for folder_path in folder_paths:
        images = sorted(
            [os.path.join(folder_path, img) for img in os.listdir(folder_path) if img.endswith(".png")],
            key=natural_sort_key
        )
        image_lists.append(images)

    # Ensure all folders have the same number of images
    min_length = min(len(images) for images in image_lists)
    image_lists = [images[:min_length] for images in image_lists]

    if min_length == 0:
        print("No PNG images found in one or more folders.")
        return

    # Read the first image to determine the video frame size
    sample_image = cv2.imread(image_lists[0][0])
    height, width, _ = sample_image.shape
    border_width = 16
    total_width = len(image_lists) * width + (len(image_lists) - 1) * border_width

    # Initialize the video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Codec for MP4
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (total_width, height))

    
    # Iterate over the images and concatenate them
    for i, frames in enumerate(zip(*image_lists)):
        print(i)
        concatenated_frame = None
        for i, image_path in enumerate(frames):
            frame = cv2.imread(image_path)
            # Invert the images for the first and last folders
            if i == 0 or i == len(frames) - 1:
                frame = cv2.flip(frame, 1)  # Horizontal flip
            if concatenated_frame is None:
                concatenated_frame = frame
            else:
                concatenated_frame = np.hstack((concatenated_frame, np.zeros((height, border_width, 3), dtype=np.uint8), frame))
        video_writer.write(concatenated_frame)

    # Release the video writer
    video_writer.release()
    print(f"Video saved as {output_file}")

# Example usage
if __name__ == "__main__":
    start_time = time.time()
    input_folders = ["recorded_data/left", "recorded_data/windshield", "recorded_data/right"]  # Replace with your folder paths
    output_video = "output_video_wide_flipped.mp4"    # Replace with your desired output file name
    pngs_to_mp4(input_folders, output_video)
    end_time = time.time()
    print(f"Time taken: {end_time - start_time:.2f} seconds")



