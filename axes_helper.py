import time

import numpy as np


class AxesHelper:
    def __init__(self, axes_min_max_values, n_bins=3, memory_size=10):
        self.axes_min_max_values = axes_min_max_values
        self.n_bins = n_bins
        self.axis_bins = {}
        self.axis_mids = {}
        self._calculate_bins_and_mids()

        self.memory = []
        self.memory_size = memory_size

        self.active_focus_areas = {axis: None for axis in self.axes_min_max_values}
        self.ignore_mean_until = {axis: 0 for axis in self.axes_min_max_values}

        self.new_alignment = np.zeros(3)

        self.focus_reset_delay = 0.1

        print(f"axis mids: {self.axis_bins}")
        print(f"axis mids: {self.axis_mids}")

    def update_alignment(self, current_alignment, intersection):
        if intersection:
            new_alignment = current_alignment
        else:
            new_alignment = self.get_axes_mid_values()

        if len(self.memory) >= self.memory_size:
            self.memory.pop(0)
        self.memory.append(new_alignment)

    def get_bin_midpoint(self, axis, index):
        bins = self.axis_bins.get(axis)
        if bins is None or index < 0 or index >= len(bins) - 1:
            raise ValueError(f"Invalid index or bins for axis '{axis}'.")

        lower_edge = bins[index]
        upper_edge = bins[index + 1]

        return (lower_edge + upper_edge) / 2

    def get_bin_midpoint_index(self, axis, index):
        bins = self.axis_bins.get(axis)
        if bins is None or index < 0 or index >= len(bins) - 1:
            raise ValueError(f"Invalid index or bins for axis '{axis}'.")

        # Special cases for index 0 and the last index
        if index == 0:
            return 0
        elif index == len(bins) - 2:  # Last valid midpoint index
            return 100

        # Regular case: calculate midpoint
        lower_edge = bins[index]
        upper_edge = bins[index + 1]
        return (lower_edge + upper_edge) / 2

    def _calculate_active_focus_areas(self):
        if len(self.memory) == 0:
            #     print(current_active_focus_areas[axis], distance_to_target)
            return self.active_focus_areas

        mean_alignment = np.mean(self.memory, axis=0)
        timestamp = time.time()

        for i, axis in enumerate(["x", "y", "z"]):
            current_bin = np.digitize(mean_alignment[i], self.axis_bins[axis]) - 1
            current_bin = min(max(current_bin, 0), self.n_bins - 1)
            if timestamp > self.ignore_mean_until[axis]:
                if self.active_focus_areas[axis] != current_bin:
                    self.active_focus_areas[axis] = current_bin
                    self.ignore_mean_until[axis] = timestamp + self.focus_reset_delay
                else:
                    self.ignore_mean_until[axis] = timestamp + self.focus_reset_delay

        return self.active_focus_areas

    def calculate_target_alignment(
        self, min_step_size=0.0001, max_step_size=0.005, smoothing_factor=0.5
    ):
        current_active_focus_areas = self._calculate_active_focus_areas()

        for i, axis in enumerate(["x", "y", "z"]):
            if current_active_focus_areas[axis] is not None:

                target_midpoint = self.get_bin_midpoint_index(
                    axis, current_active_focus_areas[axis]
                )

                distance_to_target = target_midpoint - self.new_alignment[i]

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

    def _calculate_bins_and_mids(self):
        for axis, (min_val, max_val) in self.axes_min_max_values.items():
            bins = np.linspace(min_val, max_val, self.n_bins + 1)
            self.axis_bins[axis] = bins

            mid_value = (min_val + max_val) / 2
            self.axis_mids[axis] = mid_value

    def set_axes_min_max(self, new_axes_min_max_values):
        self.axes_min_max_values = new_axes_min_max_values
        self._calculate_bins_and_mids()

    def get_axes_mid_values(self):
        return np.array([self.axis_mids[axis] for axis in ["x", "y", "z"]])
