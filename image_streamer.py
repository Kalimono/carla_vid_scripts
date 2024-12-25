import os
import time
import threading
from itertools import cycle

import cv2


class ImageStreamer:
    def __init__(
        self,
        parent,
        read_into_memory=True,
        left_mirror_dir="driving_with_traffic2/left_mirror",
        right_mirror_dir="driving_with_traffic2/right_mirror",
        windshield_dir="driving_with_traffic2/windshield",
    ):
        self.parent = parent
        self.read_into_memory = read_into_memory

        self.left_images = self._load_images(left_mirror_dir)
        self.right_images = self._load_images(right_mirror_dir)
        self.windshield_images = self._load_images(windshield_dir)

        self.left_cycle = cycle(self.left_images)
        self.right_cycle = cycle(self.right_images)
        self.windshield_cycle = cycle(self.windshield_images)

        self.current_left_image = None
        self.current_right_image = None
        self.current_windshield_image = None

        self.FRAME_INTERVAL = 1 / 30
        self.last_frame_time = time.time()


    def _load_images(self, directory):
        if not os.path.isdir(directory):
            raise ValueError(f"Directory not found: {directory}")

        if self.read_into_memory:
            return [
                cv2.imread(os.path.join(directory, file))
                for file in sorted(os.listdir(directory)[:300])
                if file.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tiff"))
            ]

        return [
            os.path.join(directory, file)
            for file in sorted(os.listdir(directory))
            if file.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tiff"))
        ]

    def _update_images(self):
        self.current_left_image = next(self.left_cycle)
        self.current_right_image = next(self.right_cycle)
        self.current_windshield_image = next(self.windshield_cycle)

        self.parent.update_mirror_frames(
            (self.current_left_image, self.current_right_image, self.current_windshield_image)
        )
        # self.parent.update_windshield_frame(self.current_windshield_image)

    def main(self):
        # def stream_images():
        while True:
            now = time.time()
            # # if mirror_name == "Left Mirror":
            # #     print(f"Left mirror:{now - self.last_frame_time[mirror_name]}")
            
            if now - self.last_frame_time < self.FRAME_INTERVAL:
                print(self.FRAME_INTERVAL - (now - self.last_frame_time))
                time.sleep(self.FRAME_INTERVAL - (now - self.last_frame_time))
            self.last_frame_time = time.time()

            self._update_images()
        # threading.Thread(target=stream_images, daemon=True).start()
