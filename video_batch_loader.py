import cv2
import numpy as np
from multiprocessing import shared_memory
from threading import Thread, Lock
from queue import Queue
import time


class VideoBatchLoader:
    def __init__(self, video_path, batch_size):
        """
        Initialize the video batch loader.
        
        Args:
            video_path (str): Path to the video file.
            batch_size (int): Number of frames per batch.
        """
        self.video_path = video_path
        self.batch_size = batch_size
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise ValueError("Could not open video file")
        
        # Video properties
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.channels = 3  # Assuming color video (BGR)

        # Shared memory allocation for batches
        self.frame_size = self.frame_height * self.frame_width * self.channels
        self.batch_size_bytes = self.frame_size * self.batch_size
        self.shm = shared_memory.SharedMemory(create=True, size=self.batch_size_bytes)
        self.batch_array = np.ndarray(
            (self.batch_size, self.frame_height, self.frame_width, self.channels),
            dtype=np.uint8,
            buffer=self.shm.buf
        )
        
        # Queue for frames and synchronization
        self.frame_queue = Queue(maxsize=self.batch_size)
        self.lock = Lock()
        self.running = True
        self.loader_thread = Thread(target=self._load_batches)
        self.loader_thread.start()

    def _load_batches(self):
        """
        Private method to load frames into the shared memory buffer asynchronously.
        """
        while self.running:
            if self.frame_queue.qsize() < self.batch_size // 2:  # Load if half or more frames are consumed
                with self.lock:
                    frames_in_batch = 0
                    for i in range(self.batch_size):
                        ret, frame = self.cap.read()
                        if not ret:
                            self.running = False
                            break
                        self.batch_array[i] = frame
                        self.frame_queue.put(i)
                        frames_in_batch += 1

                    if frames_in_batch == 0:
                        break

    def get_next_frame(self):
        """
        Public method to get the next frame.
        
        Returns:
            np.ndarray: The next frame in the sequence.
        """
        if not self.running and self.frame_queue.empty():
            raise StopIteration("No more frames available")
        
        frame_index = self.frame_queue.get()
        with self.lock:
            return self.batch_array[frame_index].copy()

    def reset(self):
        """
        Reset the video loader to the beginning of the video.
        """
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.running = True
        self.loader_thread = Thread(target=self._load_batches)
        self.loader_thread.start()

    def cleanup(self):
        """
        Release resources and clean up shared memory.
        """
        self.running = False
        self.loader_thread.join()
        self.cap.release()
        self.shm.close()
        self.shm.unlink()

    def __del__(self):
        """
        Ensure resources are cleaned up on deletion.
        """
        self.cleanup()

video_path = "output_video_wide_flipped.mp4"
batch_size = 10

# Initialize the VideoBatchLoader
video_loader = VideoBatchLoader(video_path, batch_size)

counter = 0

try:

    while True:
        try:
            frame = video_loader.get_next_frame()
            # print(f"Got frame with shape: {frame.shape}")
            print(f"Frame {counter} processed")
            counter += 1
            time.sleep(0.1)

            # Process the frame
        except StopIteration:
            print("End of video reached")
            break
finally:
    video_loader.cleanup()
