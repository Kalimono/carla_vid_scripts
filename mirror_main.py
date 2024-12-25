import threading

import time

import numpy as np

from camera_stream import CameraStream
from mirror_logic import MirrorLogic
from image_streamer import ImageStreamer
from windshield_streamer import WindshieldStreamer
from mp4_reader import MP4Reader

import threading


class MirrorMain:
    def __init__(self):
        self.camera_stream = CameraStream()
        # self.camera_stream_right = CameraStream()
        self.mirror_logic = MirrorLogic()

        self.windshield_streamer = WindshieldStreamer()

        # self.image_streamer = ImageStreamer(self)

        # self.mp4_reader = MP4Reader(self, "output_video_wide_flipped.mp4")

        # self.start_camera_stream_thread_left()
        self.start_camera_stream_thread()
        # self.start_windshield_stream_thread()

        # self.start_image_streamer_thread()

        # self.start_mp4_reader_thread()

    def debug_thread_location(self, func_name):
        print(
            f"[DEBUG] {func_name} called from thread: {threading.current_thread().name}"
        )

    def update_mirror_frames(self, frames):
        self.camera_stream.update_mirror_frame(frames)

    # def update_right_mirror_frame(self, frame):
    #     self.camera_stream_right.update_mirror_frame(frame)

    def update_windshield_frame(self, frame):
        self.windshield_streamer.update_image(frame)

    def start_camera_stream(self):
        self.camera_stream.start_stream()

    # def start_camera_stream_right(self):
    #     self.camera_stream_right.start_stream()

    def start_windshield_stream(self):
        self.windshield_streamer.run()

    def start_image_streamer(self):
        self.image_streamer.main()

    def start_mp4_reader(self):
        self.mp4_reader.main()

    def start_camera_stream_thread(self):
        self.camera_thread_left = threading.Thread(target=self.start_camera_stream)
        self.camera_thread_left.start()

    # def start_camera_stream_thread_right(self):
    #     self.camera_thread_right = threading.Thread(target=self.start_camera_stream_right)
    #     self.camera_thread_right.start()

    def start_windshield_stream_thread(self):
        self.windshield_thread = threading.Thread(target=self.start_windshield_stream)
        self.windshield_thread.start()

    # def start_image_streamer_thread(self):
    #     self.image_thread = threading.Thread(target=self.start_image_streamer)
    #     self.image_thread.start()

    def start_mp4_reader_thread(self):
        self.mp4_thread = threading.Thread(target=self.start_mp4_reader)
        self.mp4_thread.start()

    def receive_socket_data(self, data):
        # self.debug_thread_location("receive_socket_data")
        # head_position = np.array(data["viewingDirection"]["position"])

        head_position = np.round(np.array(data["headPose"]["position"]), 2)

        distance_to_mirror = self.mirror_logic.distance_to_mirror_mid(head_position)

        # print(f"Distance to mirror: {distance_to_mirror}")

        has_left_mirror = any(
            target.get("name") == "LeftMirror"
            for target in data["viewingTargetsInfo"]["targets"]
        )

        has_right_mirror = any(
            target.get("name") == "RightMirror"
            for target in data["viewingTargetsInfo"]["targets"]
        )

        has_left_mirror = True
        has_right_mirror = True

        self.camera_stream.update_alignments(
            head_position, distance_to_mirror, has_left_mirror, has_right_mirror
        )  # has_left_mirror, has_right_mirror)
        # self.camera_stream_right.update_alignments(head_position, distance_to_mirror, not has_left_mirror)


if __name__ == "__main__":
    mirror_main = MirrorMain()
    # mirror_main.start_camera_stream_thread() 2800 3100
