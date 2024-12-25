import time

import cv2
import numpy as np
from screeninfo import get_monitors

from axes_helper import AxesHelper


class CameraStream:
    def __init__(self, fallback_image_path="vaporwave_small.png"):
        self.fallback_image_path = fallback_image_path
        self.fallback_image = None

        self.screen_width = 0
        self.screen_height = 0

        self.stream_running = False

        self.alignment_x = -1
        self.alignment_y = -1
        self.alignment_z = -1

        self.slice_width_percent = 25
        self.slice_height_percent = 50

        self.x_freeze = True
        self.y_freeze = True
        self.z_freeze = False

        self.mid_x_mapped = -1
        self.mid_y_mapped = -1
        self.mid_z_mapped = -1

        self.calibration_active = False
        self.calibration_done = False

        self.recentered = False

        self.left_mirror_index = 0

        # self.axes_min_max_values = {
        #     "x": (-0.09007611125707626, 0.0187331922352314),
        #     "y": (-0.032651565968990326, 0.008753980121885737),
        #     "z": (0.6103035122002182, 0.7832357930831002),
        # } #using closeness to mirror as reference

        self.axes_min_max_values = {
            "x": (-0.07833235710859299, 0.0004008606483694166),
            "y": (-0.04263470694422722, 0.0),
            "z": (-0.11192318797111511, 0.07438728213310242),
        }

        self.axes_helper = AxesHelper(self.axes_min_max_values)

        self.current_mirror_frame = None

        self.new_alignment = self._get_max_values()

        try:
            self.fallback_image = cv2.imread(self.fallback_image_path)
            if self.fallback_image is None:
                print(
                    f"Error: Could not load fallback image {self.fallback_image_path}"
                )
        except Exception as e:
            print(f"Exception occurred while loading fallback image: {e}")

    # def _calculate_height_width_ratio(self, image):
    #     self.slice_height_percent = 50
    #     self.slice_width_percent =

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
            min_val, max_val = self.axes_min_max_values[axis]
            original_range = max_val - min_val

            new_min = current_alignment[i] - min_val
            new_max = current_alignment[i] - max_val

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

    def update_alignments(self, alignment_vector, distance_to_mirror, viewingTarget):

        # alignment_vector = np.array([alignment_vector[0], alignment_vector[1], -distance_to_mirror])

        if self.calibration_active:
            self._extend_max_min_values(-alignment_vector)

        if viewingTarget:
            self.alignment_x = -alignment_vector[0]
            self.alignment_y = -alignment_vector[1]
            self.alignment_z = -alignment_vector[2]

            if not self.recentered:
                print("Recentering axes")
                self._recenter_axes_min_max_values(-alignment_vector)
                self.recentered = True
        else:
            if self.recentered:
                self.recentered = False
                self.new_alignment = self._get_max_values()
            (
                self.alignment_x,
                self.alignment_y,
                self.alignment_z,
            ) = self._calculate_target_alignment()

        # (
        #     self.alignment_x,
        #     self.alignment_y,
        #     self.alignment_z,
        # ) = -alignment_vector

        # self.axes_helper.update_alignment(-alignment_vector, viewingTarget)
        # (
        #     self.alignment_x,
        #     self.alignment_y,
        #     self.alignment_z,
        # ) = self.axes_helper.calculate_target_alignment()

    def update_mirror_frame(self, image):
        # if np_array is None:
        #     return

        # self.current_mirror_frame = self._numpy_to_image_pil(np_array)

        self.current_mirror_frame = cv2.imread(image)

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

        cv2.rectangle(
            fov_region,
            (rect_x, rect_y),
            (rect_x + rect_w, rect_y + rect_h),
            (0, 0, 255),
            2,
        )

        # cv2.imshow("FOV", fov_region)

        # print(f"Ratio after: {fov_region.shape[1] / fov_region.shape[0]}")

        compressed_slice = cv2.resize(fov_region, (slice_width, slice_height))

        # print(slice_width, slice_height)

        return fov_region

    def _run_stream(self):
        is_fullscreen = False
        window_name = "Square Slice Stream"
        cv2.namedWindow(window_name, flags=cv2.WINDOW_GUI_NORMAL)

        monitor = get_monitors()[0]
        self.screen_width = monitor.width
        self.screen_height = monitor.height

        while self.stream_running:
            frame = self.current_mirror_frame

            if frame is None or frame.size == 0:
                if self.fallback_image is not None:
                    frame = self.fallback_image.copy()
                else:
                    print("Error: No camera feed or fallback image available")
                    break

            if not self.x_freeze:
                width_percentage = self._percentage_between(
                    self.alignment_x,
                    self.axes_min_max_values["x"][0],
                    self.axes_min_max_values["x"][1],
                )
            else:
                width_percentage = self.mid_x_mapped

            if not self.y_freeze:
                height_percentage = self._percentage_between(
                    self.alignment_y,
                    self.axes_min_max_values["y"][0],
                    self.axes_min_max_values["y"][1],
                )
            else:
                height_percentage = self.mid_y_mapped

            if not self.z_freeze:
                depth_percentage = self._percentage_between(
                    self.alignment_z,
                    self.axes_min_max_values["z"][0],
                    self.axes_min_max_values["z"][1],
                )
            else:
                depth_percentage = self.mid_z_mapped

            frame_height, frame_width, _ = frame.shape

            # cv2.imshow("Original", frame)

            slice_width = int(frame_width * self.slice_width_percent / 100)
            slice_height = int(frame_height * self.slice_height_percent / 100)

            square_slice = self._increase_fov(
                frame,
                slice_width,
                slice_height,
                depth_percentage,
            )

            slice_aspect_ratio = square_slice.shape[1] / square_slice.shape[0]
            screen_aspect_ratio = self.screen_width / self.screen_height

            # print(square_slice.shape, slice_aspect_ratio)

            if slice_aspect_ratio > screen_aspect_ratio:
                target_width = self.screen_width
                target_height = int(self.screen_width / slice_aspect_ratio)
            else:
                target_height = self.screen_height
                target_width = int(self.screen_height * slice_aspect_ratio)

            # resized_slice = square_slice[:slice_height, :slice_width]

            # cv2.imshow("Slice", resized_slice)

            resized_slice = cv2.resize(square_slice, (target_width, target_height))

            # resized_slice = cv2.cvtColor(resized_slice, cv2.COLOR_BGR2RGB) cool

            background = np.zeros(
                (self.screen_height, self.screen_width, 3), dtype=np.uint8
            )
            x_offset = (self.screen_width - target_width) // 2
            y_offset = (self.screen_height - target_height) // 2
            background[
                y_offset : y_offset + target_height, x_offset : x_offset + target_width
            ] = resized_slice

            cv2.imshow(window_name, background)

            key = cv2.waitKey(1)
            if key == ord("f"):
                is_fullscreen = not is_fullscreen
                cv2.setWindowProperty(
                    window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if is_fullscreen else cv2.WINDOW_NORMAL,
                )

            if key == ord("q"):
                self.stream_running = False
                break

            if key == ord("r"):
                self._set_zero_alignment(self.current_alignment_angle)

            if key == ord("x"):
                self.x_freeze = not self.x_freeze
                print(f"X Freeze: {self.x_freeze}")

            if key == ord("y"):
                self.y_freeze = not self.y_freeze
                print(f"Y Freeze: {self.y_freeze}")

            if key == ord("z"):
                self.z_freeze = not self.z_freeze
                print(f"Z Freeze: {self.z_freeze}")

            if key == ord("c"):
                self.calibration_active = not self.calibration_active
                print(f"Calibration mode: {self.calibration_active}")

            if key == ord("v"):
                self._set_min_max_to_current(
                    (self.alignment_x, self.alignment_y, self.alignment_z)
                )
                print("Calibration values set to current alignment")
            time.sleep(0.01)

        cv2.destroyAllWindows()

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
