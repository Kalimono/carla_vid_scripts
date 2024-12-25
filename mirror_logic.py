import math

import numpy as np


class MirrorLogic:
    def __init__(self):
        self.worldmodel = {}

    def set_worldmodel(self, worldmodel):
        self.worldmodel = worldmodel

    def compute_angle_between_vectors(self, viewing_direction_point):
        lowerLeft1 = np.array([-0.778, 0.014, 1.231])
        xAxis1 = np.array([-0.999, 0.039, -0.002])
        yAxis1 = np.array([0.003, 0.999, -0.037])
        size1 = np.array([0.185, 0.292])

        lowerLeft1 = np.array([0.426, -0.299, 0.571])
        xAxis1 = np.array([-0.930, 0.125, 0.345])
        yAxis1 = np.array([0.025, 0.978, -0.205])
        size1 = np.array([0.208, 0.346])

        xAxis_scaled1 = xAxis1 * size1[0]
        yAxis_scaled1 = yAxis1 * size1[1]

        P1_1 = lowerLeft1
        P2_1 = P1_1 + xAxis_scaled1
        P3_1 = P1_1 + yAxis_scaled1
        P4_1 = P1_1 + xAxis_scaled1 + yAxis_scaled1

        M1 = (P1_1 + P2_1 + P3_1 + P4_1) / 4

        v1_1 = P2_1 - P1_1
        v2_1 = P3_1 - P1_1
        N1 = np.cross(v1_1, v2_1)

        N_unit1 = N1 / np.linalg.norm(N1)

        direction_vector_M1_to_viewing = viewing_direction_point - M1

        direction_vector_M1_to_viewing_unit = (
            direction_vector_M1_to_viewing
            / np.linalg.norm(direction_vector_M1_to_viewing)
        )

        vector_difference = direction_vector_M1_to_viewing_unit - N_unit1

        dot_product = np.dot(direction_vector_M1_to_viewing_unit, N_unit1)
        angle_between_vectors = np.arccos(dot_product)

        angle_in_degrees = np.degrees(angle_between_vectors)

        # print(f"Direction vector (unit) from M1 to viewing direction point: {direction_vector_M1_to_viewing_unit}")
        # print(f"Vector difference: {vector_difference}")
        # print(f"Angle between normal vector and direction vector: {angle_in_degrees} degrees")

        return angle_in_degrees, vector_difference

    def distance_3d(p1, p2):
        return math.sqrt(
            (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2
        )

    def distance_to_mirror_mid(self, viewing_direction_point):
        lowerLeft1 = np.array([0.426, -0.299, 0.571])
        xAxis1 = np.array([-0.930, 0.125, 0.345])
        yAxis1 = np.array([0.025, 0.978, -0.205])
        size1 = np.array([0.208, 0.346])

        xAxis_scaled1 = xAxis1 * size1[0]
        yAxis_scaled1 = yAxis1 * size1[1]

        P1_1 = lowerLeft1
        P2_1 = P1_1 + xAxis_scaled1
        P3_1 = P1_1 + yAxis_scaled1
        P4_1 = P1_1 + xAxis_scaled1 + yAxis_scaled1

        M1 = (P1_1 + P2_1 + P3_1 + P4_1) / 4

        distance = np.linalg.norm(viewing_direction_point - M1)

        return distance
