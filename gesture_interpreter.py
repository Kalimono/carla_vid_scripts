import os
import time

import pygame


class GestureInterpreter:
    def __init__(self, directory="voice_files"):
        pygame.mixer.init()

        self.sounds = {}
        for file in os.listdir(directory):
            if file.endswith(".mp3"):
                file_path = os.path.join(directory, file)
                self.sounds[file] = file_path

        self.last_gesture_time = 0
        self.cooldown = 5

    def play_sound(self, name):
        if name in self.sounds:
            pygame.mixer.music.load(self.sounds[name])
            pygame.mixer.music.play()
        else:
            print(f"Sound {name} not found!")

    def interpret_gestures(self, hand_gest_1, hand_gest_2):
        current_time = time.time()

        if current_time - self.last_gesture_time < self.cooldown:
            print("Cooldown in effect. Try again later.")
            return -1

        if hand_gest_1 == 0 and hand_gest_2 == 0:  # THUMBS_UP
            self.play_sound("calibration_activated.mp3")
            self.last_gesture_time = current_time
            return 1
        if hand_gest_1 == 1 and hand_gest_2 == 1:  # THUMBS_DOWN
            self.play_sound("calibration_deactivated.mp3")
            self.last_gesture_time = current_time
            return 2
        elif (hand_gest_1 == 2 and hand_gest_2 == 4) or (
            hand_gest_1 == 4 and hand_gest_2 == 2
        ):  # OPEN_PALM and PEACE_SIGN
            self.play_sound("current_pos_zeroed.mp3")
            self.last_gesture_time = current_time
            return 3
        else:
            return -1
