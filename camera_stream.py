import time

import os
import cv2
import numpy as np
from screeninfo import get_monitors

from axes_helper import AxesHelper

import pygame

import tkinter as tk
from PIL import Image, ImageTk

import multiprocessing
from multiprocessing import shared_memory

from mp4_reader import MP4Reader

class CameraStream:
    def __init__(self, fallback_image_path="fallback.jpg"):
        self.fallback_image_path = fallback_image_path
        self.fallback_image = None

        self.mirror_width = 0
        self.mirror_height = 0

        self.windshield_screen_width = 0
        self.windshield_screen_height = 0

        self.stream_running = False

        self.alignment_x = -1
        self.alignment_y = -1
        self.alignment_z = -1

        self.slice_width_percent = 25
        self.slice_height_percent = 100

        self.x_freeze = False
        self.y_freeze = True
        self.z_freeze = False

        self.mid_x_mapped = -1
        self.mid_y_mapped = -1
        self.mid_z_mapped = -1

        self.windshield_index = 0
        self.left_mirror_index = 0
        self.right_mirror_index = 0

        self.calibration_active = False
        self.calibration_done = False

        self.recentered = False
        self.last_view_time = time.time()

        self.runtime_means = []

        self.zoom = False

        self.mp4_reader = MP4Reader(self, "output_video_wide_flipped.mp4")
        # self.FRAME_INTERVAL = 1 / 60
        # self.last_frame_time = {"Left Mirror": time.time(), "Right Mirror":time.time()}

        # self.axes_min_max_values = {'x': (-0.07833235710859299, 0.0004008606483694166), 'y': (-0.04263470694422722, 0.0), 'z': (-0.11192318797111511, 0.07438728213310242)}

        # self.axes_min_max_values = {
        #     "x": (-0.07833235710859299, 0.0004008606483694166),
        #     "y": (-0.04263470694422722, 0.0),
        #     "z": (-0.05, 0.1),
        # }

        self.axes_min_max_values = {
            "x": (-0.21, 0.19),
            "y": (-0.08, 0.12),
            "z": (-0.05, 0.1),
        }

        self.axes_helper = AxesHelper(self.axes_min_max_values)

        self.current_mirror_frames_dict = {"left": None, "right": None, "windshield": None}

        self.new_alignment = self._get_max_values()

        self.alignment_memory = []

        try:
            self.fallback_image = cv2.imread(self.fallback_image_path)
            if self.fallback_image is None:
                print(
                    f"Error: Could not load fallback image {self.fallback_image_path}"
                )
        except Exception as e:
            print(f"Exception occurred while loading fallback image: {e}")

    def update_runtime_means(self, runtime):
        self.runtime_means.append(runtime)
        
        if len(self.runtime_means) > 1000:
            self.runtime_means.pop(0)

        # print(self.runtime_means)


    def get_runtime_mean(self):
        return np.mean(self.runtime_means)

    def _update_memory(self, alignment_vector):
        self.alignment_memory.append(alignment_vector)
        if len(self.alignment_memory) > self.memory_length:
            self.alignment_memory.pop(0)

    def _get_memory_average(self):
        return np.mean(self.alignment_memory, axis=0)


    def _numpy_to_image_pil(self, np_array):
        bgr_frame = cv2.cvtColor(np_array, cv2.COLOR_RGB2BGR)
        return bgr_frame

    def _set_min_value(self, axis, value):
        self.axes_min_max_values[axis] = (value, self.axes_min_max_values[axis][1])

    def _set_max_value(self, axis, value):
        self.axes_min_max_values[axis] = (self.axes_min_max_values[axis][0], value)

    def _extend_max_min_values(self, alignment):
        for i, axis in enumerate(["x", "y", "z"]):
            if alignment[i] < self.axes_min_max_values[axis][0]:
                self._set_min_value(axis, alignment[i])
            if alignment[i] > self.axes_min_max_values[axis][1]:
                self._set_max_value(axis, alignment[i])

        print(self.axes_min_max_values)

        self.axes_helper.set_axes_min_max(self.axes_min_max_values)

    def _reset_max_min_values(self):
        self.axes_min_max_values = {"x": (0, 0), "y": (0, 0), "z": (0, 0)}

    def _set_min_max_to_current(self, alignment):
        self._reset_max_min_values()
        for i, axis in enumerate(["x", "y", "z"]):
            self._set_min_value(axis, alignment[i])
            self._set_max_value(axis, alignment[i])

    def _get_max_values(self):
        return [self.axes_min_max_values[axis][1] for axis in ["x", "y", "z"]]

    def _recenter_axes_min_max_values(self, current_alignment):
        for i, axis in enumerate(["x", "y", "z"]):

            if axis == "z":
                offset = 0.1
            else:
                offset = 0
            min_val, max_val = self.axes_min_max_values[axis]
            original_range = max_val - min_val


            new_min = current_alignment[i] - original_range * .5 - offset
            new_max = current_alignment[i] + original_range * .5 - offset

            self.axes_min_max_values[axis] = (new_min, new_max)

        self.axes_helper.set_axes_min_max(self.axes_min_max_values)

    def _calculate_target_alignment(
        self, min_step_size=0.0001, max_step_size=0.001, smoothing_factor=0.1
    ):
        # current_active_focus_areas = self._calculate_active_focus_areas()

        for i, axis in enumerate(["x", "y", "z"]):
            # if current_active_focus_areas[axis] is not None:

            target_midpoint = self._get_max_values()[i]

            distance_to_target = (
                target_midpoint
                - np.array([self.alignment_x, self.alignment_y, self.alignment_z])[i]
            )

            # if axis == "z":
            #     print(current_active_focus_areas[axis], target_midpoint)

            step_size = np.clip(
                abs(distance_to_target) * smoothing_factor,
                min_step_size,
                max_step_size,
            )

            if abs(distance_to_target) > step_size:
                self.new_alignment[i] += np.sign(distance_to_target) * step_size
            else:
                self.new_alignment[i] = target_midpoint

        return self.new_alignment

    def update_alignments(
            self, alignment_vector, distance_to_mirror, viewing_left, viewing_right
        ):  
            # print(f"Alignment vector: {alignment_vector}")
            self.viewing_left = viewing_left
            self.viewing_right = viewing_right

            if self.calibration_active:
                self._extend_max_min_values(-alignment_vector)

            if viewing_left or viewing_right:
                self.last_view_time = time.time()  # Update the timer
                self.alignment_x = -alignment_vector[0]
                self.alignment_y = -alignment_vector[1]
                self.alignment_z = -alignment_vector[2]

                if not self.recentered:
                    # print("Recentering axes")
                    # self._recenter_axes_min_max_values(
                    #     np.array([self.alignment_x, self.alignment_y, self.alignment_z])
                    # )
                    self.recentered = True
            else:
                # print(time.time() - self.last_view_time)
                # Check if two seconds have passed since the last view time
                if time.time() - self.last_view_time >= 5:
                    if self.recentered:
                        self.recentered = False
                        self.new_alignment = self._get_max_values()
                    (
                        self.alignment_x,
                        self.alignment_y,
                        self.alignment_z,
                    ) = self._calculate_target_alignment()


    def _percentage_between(self, value, low, high):
        

        if high == -1 or low == -1:
            # print("Error: High or low value is -1")
            return 50.0

        if high == 0 or low == 0:
            # print("Error: High or low value is 0")
            return 0.0

        if value < low:
            # print("Error: Value is lower than low value")
            return 0.0
        elif value > high:
            # print("Error: Value is higher than high value")
            return 100.0

        percentage = ((value - low) / (high - low)) * 100

        if np.isnan(percentage):
            # print("Error: Percentage is NaN")
            return 50.0

        # print(percentage, value, low, high)
        return percentage

    def _map_value(
        self, value, target_high, target_low=0, original_high=100, original_low=0
    ):
        if value < original_low:
            return target_low
        elif value > original_high:
            return target_high

        mapped_value = target_low + (
            (value - original_low) / (original_high - original_low)
        ) * (target_high - target_low)

        if np.isnan(mapped_value):
            return 0

        return int(mapped_value)

    def _increase_fov(
        self,
        image,
        slice_width,
        slice_height,
        fov_percentage,
        midpoint_percentage_x=0,
        midpoint_percentage_y=50,
    ):  
        height, width = image.shape[:2]

        fov_percentage = 200 - fov_percentage

        fov_height = int(slice_height * fov_percentage / 100)

        if height < fov_height:

            fov_percentage = height / slice_height * 100
            fov_height = height

        midpoint_y = int(height * midpoint_percentage_y / 100)

        start_y = int(max(0, midpoint_y - fov_height // 2))
        end_y = int(min(height, midpoint_y + fov_height // 2))
        start_y = max(0, end_y - fov_height)

        fov_region = image[start_y:end_y, :]

        fov_width = int(slice_width * fov_percentage / 100)

        midpoint_x = int(width * midpoint_percentage_x / 100)

        start_x = int(max(0, midpoint_x - fov_width // 2))

        end_x = int(min(width, midpoint_x + fov_width))
        start_x = max(0, end_x - fov_width)
        fov_region = fov_region[:, start_x:end_x]

        aspect_ratio = slice_width / slice_height

        zoomed_in_height = slice_height
        zoomed_in_width = int(zoomed_in_height * aspect_ratio)

        zoom_start_x = int(max(0, midpoint_x - zoomed_in_width // 2))
        zoom_end_x = int(min(width, midpoint_x + zoomed_in_width))
        zoom_start_x = max(0, zoom_end_x - zoomed_in_width)

        zoom_start_y = int(max(0, midpoint_y - zoomed_in_height // 2))
        zoom_end_y = int(min(height, midpoint_y + zoomed_in_height // 2))
        zoom_start_y = max(0, zoom_end_y - zoomed_in_height)

        fov_region = image[start_y:end_y, start_x:end_x].copy()
        
        rect_x = max(0, zoom_start_x - start_x)
        rect_y = max(0, zoom_start_y - start_y)
        rect_w = min(zoom_end_x - zoom_start_x, fov_region.shape[1] - rect_x)
        rect_h = min(zoom_end_y - zoom_start_y, fov_region.shape[0] - rect_y)

        offset = 5
        cv2.rectangle(
            fov_region,
            (rect_x - offset, rect_y - offset),
            (rect_x + rect_w + offset, rect_y + rect_h + offset),
            (0, 0, 255),
            2,
        )

        return fov_region
    
    def _calculate_slice_bounds(
        self,
        image,
        slice_width,
        slice_height,
        midpoint_percentage_x,
        midpoint_percentage_y,
    ):
        height, width, _ = image.shape

        # print(f"Midpoint x: {midpoint_percentage_x}, Midpoint y: {midpoint_percentage_y}")

        mid_x_mapped = self._map_value(
            midpoint_percentage_x,
            width,
        )

        mid_y_mapped = self._map_value(
            midpoint_percentage_y,
            height,
        )

        # print(f"Slice width: {slice_width}, Slice height: {slice_height}")

        start_x = mid_x_mapped - slice_width // 2
        end_x = mid_x_mapped + slice_width // 2

        start_y = mid_y_mapped - slice_height // 2
        end_y = mid_y_mapped + slice_height // 2

        if end_x > width:
            end_x = width
            start_x = end_x - slice_width
        elif start_x < 0:
            start_x = 0
            end_x = start_x + slice_width

        if end_y > height:
            end_y = height
            start_y = end_y - slice_height
        elif start_y < 0:
            start_y = 0
            end_y = start_y + slice_height

        # print(f"Start x: {start_x}, End x: {end_x}, Start y: {start_y}, End y: {end_y}")

        new_slice = image[start_y:end_y, start_x:end_x]

        return new_slice

    
    def create_shared_frame(self, name, shape, dtype=np.uint8):
        shm = shared_memory.SharedMemory(create=True, size=np.prod(shape) * np.dtype(dtype).itemsize)
        array = np.ndarray(shape, dtype=dtype, buffer=shm.buf)
        return shm, array


    def update_shared_frame(self, shared_array, frame, lock):
        # print(f"Updating shared frame with shape {frame.shape}")
        if frame.shape != shared_array.shape:
            frame = cv2.resize(frame, (shared_array.shape[1], shared_array.shape[0]))
        with lock:
            # lock_time = time.time()
            np.copyto(shared_array, frame)
            # print(f"Lock time: {time.time() - lock_time}")


    # def update_mirror_frame(self, frames):
    #     self.current_mirror_frames_dict["left"] = frames[0]
    #     self.current_mirror_frames_dict["right"] = frames[2]
    #     self.current_mirror_frames_dict["windshield"] = frames[1]

    def _run_pygame_window(self, mirror_name, shm_name, shape, lock, event, monitor_index):
        try:
            monitors = get_monitors()
            position = (monitors[monitor_index].x, monitors[monitor_index].y)
            os.environ['SDL_VIDEO_WINDOW_POS'] = f"{position[0]},{position[1]}"

            pygame.init()
            screen = pygame.display.set_mode(
                (monitors[monitor_index].width, monitors[monitor_index].height),
                pygame.SCALED | pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF
            )
            pygame.display.set_caption(mirror_name)

            shm = shared_memory.SharedMemory(name=shm_name)
            # shared_array = np.ndarray(shape, dtype=np.uint8, buffer=shm.buf)

            running = True
            surface = None

            while running:
                for event in pygame.event.get([pygame.QUIT]):
                    running = False

                event.wait()
                with lock:
                    frame = np.ndarray(shape, dtype=np.uint8, buffer=shm.buf)

                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                if surface is None or surface.get_size() != (shape[1], shape[0]):
                    surface = pygame.Surface((shape[1], shape[0]))
                pygame.surfarray.blit_array(surface, frame.swapaxes(0, 1))
                screen.blit(surface, (0, 0))

                pygame.display.flip()
        finally:

            pygame.quit()
            shm.close()
            shm.unlink()

    def _run_stream(self):
        is_fullscreen = False

        window_flags = pygame.FULLSCREEN if is_fullscreen else 0
        monitor = get_monitors()[self.left_mirror_index]

        self.mirror_width = monitor.width
        self.mirror_height = monitor.height

        windshield_monitor = get_monitors()[self.windshield_index]
        self.windshield_screen_width = windshield_monitor.width
        self.windshield_screen_height = windshield_monitor.height

        screen_aspect_ratio = self.mirror_width / self.mirror_height

        slice_height = int(self.mirror_height * 0.4)

        slice_width = int(slice_height * screen_aspect_ratio)

        left_lock = multiprocessing.Lock()
        right_lock = multiprocessing.Lock()
        windshield_lock = multiprocessing.Lock()

        left_event = multiprocessing.Event()
        right_event = multiprocessing.Event()
        windshield_event = multiprocessing.Event()

        left_shape = (1080, 1920, 3)  # Example resolution
        right_shape = (1080, 1920, 3)
        windshield_shape = (1080, 1920, 3)

        left_shm, left_array = self.create_shared_frame("left_frame", left_shape)
        right_shm, right_array = self.create_shared_frame("right_frame", right_shape)
        windshield_shm, windshield_array = self.create_shared_frame("windshield_frame", windshield_shape)

        left_process = multiprocessing.Process(
            target=self._run_pygame_window,
            args=("Left Mirror", left_shm.name, left_shape, left_lock, left_event, self.left_mirror_index)
        )
        right_process = multiprocessing.Process(
            target=self._run_pygame_window,
            args=("Right Mirror", right_shm.name, right_shape, right_lock, right_event, self.right_mirror_index)
        )
        windshield_process = multiprocessing.Process(
            target=self._run_pygame_window,
            args=("Windshield", windshield_shm.name, windshield_shape, windshield_lock, windshield_event, self.windshield_index)
        )

        left_process.start()
        right_process.start()
        windshield_process.start()

        end_time = time.time()
        name_dict = {0: "left", 2: "right", 1: "windshield"}

        # self._recenter_axes_min_max_values([self.alignment_x, self.alignment_y, self.alignment_z])

        while self.stream_running:
            

            # for name, frame in self.current_mirror_frames_dict.items():
            for frame_n, frame in self.mp4_reader:
                # print(f"Frame number: {frame_n}")
                for n, frame in enumerate(frame):
                    name = name_dict[n]
                    if frame is None or frame.size == 0:
                        if self.fallback_image is not None:
                            frame = self.fallback_image.copy()
                        else:
                            print("Error: No camera feed or fallback image available")
                            continue

                    # frame = self.fallback_image

                    if name == "windshield":
                        self.update_shared_frame(windshield_array, frame, windshield_lock)
                        windshield_event.set()
                        windshield_event.clear()
                        continue

                    # if name == "left" and not self.viewing_left:
                    #     self.midpoint_percentage_x = 100
                    # else:
                    #     self.midpoint_percentage_x = 0

                    if not self.z_freeze:
                        depth_percentage = self._percentage_between(
                            self.alignment_z,
                            self.axes_min_max_values["z"][0],
                            self.axes_min_max_values["z"][1],
                        )
                    else:
                        depth_percentage = self.mid_z_mapped

                    if not self.x_freeze:
                        self.midpoint_percentage_x = self._percentage_between(
                            self.alignment_x,
                            self.axes_min_max_values["x"][0],
                            self.axes_min_max_values["x"][1],
                        )
                    else:
                        self.midpoint_percentage_x = self.mid_x_mapped

                    if not self.y_freeze:
                        self.midpoint_percentage_y = self._percentage_between(
                            self.alignment_y,
                            self.axes_min_max_values["y"][0],
                            self.axes_min_max_values["y"][1],
                        )
                    else:
                        self.midpoint_percentage_y = self.mid_y_mapped

                    if depth_percentage == -1:
                        depth_percentage = 100.0

                    if name == "left":
                        self.midpoint_percentage_x = min(0, self.midpoint_percentage_x-50)

                    if name == "right":
                        self.midpoint_percentage_x = min(100, self.midpoint_percentage_x+50)

                    if self.zoom:
                        square_slice = self._increase_fov(
                            frame,
                            slice_width,
                            slice_height,
                            depth_percentage,
                        )
                    else:
                        # print(f"Midpoint x: {self.midpoint_percentage_x}")
                        # print(f"Axes min max values: {self.axes_min_max_values}")'

                        square_slice = self._calculate_slice_bounds(
                            frame,
                            slice_width,
                            slice_height,
                            self.midpoint_percentage_x,
                            self.midpoint_percentage_y,
                        )

                    slice_aspect_ratio = square_slice.shape[1] / square_slice.shape[0]
                    screen_aspect_ratio = self.mirror_width / self.mirror_height

                    if slice_aspect_ratio > screen_aspect_ratio:
                        target_width = self.mirror_width
                        target_height = int(self.mirror_width / slice_aspect_ratio)
                    else:
                        target_height = self.mirror_height
                        target_width = int(self.mirror_height * slice_aspect_ratio)

                    resized_slice = cv2.resize(square_slice, (target_width, target_height))

                    if name == "left":
                        self.update_shared_frame(left_array, resized_slice, left_lock)
                        left_event.set()
                        left_event.clear()
                    elif name == "right":
                        self.update_shared_frame(right_array, resized_slice, right_lock)
                        right_event.set()
                        right_event.clear()

                
                if time.time() - end_time < 0.016 and (0.016 - (time.time() - end_time)) > 0:
                    # print(f"Sleep time: {0.016 - (time.time() - end_time)}")
                    try:
                        time.sleep(0.016 - (time.time() - end_time))
                    except Exception as e:
                        print(f"Error: {e}")
                else:
                    print(f"TOOK TO LONG: {time.time() - end_time}")

                # else:
                # print(f"Frame time: {time.time() - end_time}")

                # self.update_runtime_means(time.time() - start_time)

                # print(self.get_runtime_mean())

                end_time = time.time()
                
            left_process.terminate()
            right_process.terminate()
            windshield_process.terminate()

            left_shm.close()
            left_shm.unlink()
            right_shm.close()
            right_shm.unlink()
            windshield_shm.close()
            windshield_shm.unlink()

    def start_stream(self):
        if not self.stream_running:
            self.stream_running = True
            self._run_stream()

    def stop_stream(self):
        if self.stream_running:
            self.stream_running = False


if "__main__" == __name__:
    camera_stream = CameraStream()

    camera_stream.start_stream()
