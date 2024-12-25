import cv2
import time

import numpy as np
import cv2

def split_image_with_border(image, border = 16):
    _, width, _ = image.shape
    segment_width = (width - 2 * border) // 3
    
    segment1 = image[:, :segment_width, :]
    segment2 = image[:,segment_width + border: 2 * segment_width + border, :]
    segment3 = image[:, 2 * (segment_width + border): 2 * (segment_width + border) + segment_width, :]
    
    return segment1, segment2, segment3

class MP4Reader:
    def __init__(self, parent, file_path):
        self.parent = parent
        self.file_path = file_path
        self.cap = cv2.VideoCapture(file_path)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_delay = 1 / self.fps
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.frame_count = 0

        self.current_frame = 0
        
    def __iter__(self):
        return self

    def __next__(self):
        if not self.cap.isOpened():
            raise StopIteration

        ret, frame = self.cap.read()

        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            self.frame_count = 0
            if not ret: 
                raise StopIteration

        frames = split_image_with_border(frame)
        self.frame_count += 1
        return self.frame_count, frames

    def close(self):
        if self.cap.isOpened():
            self.cap.release()

    # def main(self):
    #     try:
    #         for frame in self:
    #             frames = split_image_with_border(frame)
    #             self.parent.update_mirror_frames(frames)

    #     except Exception as e:
    #         print(e)
    #     finally:
    #         self.close()

    # def get_next_frame(self):


# if __name__ == "__main__":
#     video_file = "output_video_wide_flipped.mp4"  
    
#     try:
#         reader = MP4Reader(video_file)
#         for frame in reader:
#             frames = split_image_with_border(frame)
#             cv2.imshow("Segment 1", frames[0])
#             cv2.imshow("Segment 2", frames[1])
#             cv2.imshow("Segment 3", frames[2])
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#     except Exception as e:
#         print(e)
#     finally:
#         reader.close()
#         cv2.destroyAllWindows()
