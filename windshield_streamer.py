import cv2
import numpy as np
from screeninfo import get_monitors

import pygame

import time


class WindshieldStreamer:
    def __init__(self, monitor_index=0, width=1920, height=1080):
        self.monitor_index = monitor_index
        self.width = width
        self.height = height
        self.fullscreen = True
        self.current_image = cv2.imread("fallback.jpg")
        if self.current_image is None:
            raise FileNotFoundError("Image 'vaporwave_small.png' not found!")
        self.current_image = cv2.resize(self.current_image, (self.width, self.height))

        self.FRAME_INTERVAL = 1 / 60
        self.last_frame_time = time.time()

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            cv2.setWindowProperty(
                "Windshield Streamer", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )
        else:
            cv2.setWindowProperty(
                "Windshield Streamer", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL
            )

    def update_image(self, new_image_path):
        new_image = cv2.imread(new_image_path)
        if new_image is None:
            raise FileNotFoundError(f"Image '{new_image_path}' not found!")
        self.current_image = cv2.resize(new_image, (self.width, self.height))

    def run(self):
        monitors = get_monitors()
        if self.monitor_index >= len(monitors):
            raise ValueError(f"Monitor index {self.monitor_index} is out of range!")
        monitor = monitors[self.monitor_index]
        position = (monitor.x, monitor.y)



        cv2.namedWindow("Windshield Streamer", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Windshield Streamer", self.width, self.height)
        cv2.moveWindow("Windshield Streamer", position[0], position[1])

        cv2.setWindowProperty(
            "Windshield Streamer", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )

        while True:

            now = time.time()
            # print(f"Windshield: {now - self.last_frame_time}")
            if now - self.last_frame_time < self.FRAME_INTERVAL:
                time.sleep(self.FRAME_INTERVAL - (now - self.last_frame_time))
            self.last_frame_time = time.time()

            cv2.imshow("Windshield Streamer", self.current_image)

            key = cv2.waitKey(30) & 0xFF
            if key == 27:  
                break
            elif key == ord("f"): 
                self.toggle_fullscreen()

            # clock.tick(60)

        cv2.destroyAllWindows()