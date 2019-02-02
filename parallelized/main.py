"""

Processes:

* I/O with FMS and LED control
    * Multithreaded implementation of:
        * FMS interactions - read/write to networktables
        * control LEDs color/pattern shown
* Target identification
* Camera streamer / overlay

Queues
    * FMS_read_data
    * Data_To_Write_To_Network
    * Target_Info

streamer:
    grab a frame
    add the cross hair (?)
    check targeting - to decide whether to add border, color
    push the frame to the network

FMS reader/writer
    read: match_started, alliance
"""
import cv2
import time
from multiprocessing import Queue, Pipe
# import our bot-specific classes
from camstreamer import Camstreamer
from targeting import Targeting


# CONSTANTS
CAMERA_SOURCE = "http://10.15.18.11/mjpg/video.mjpg"
LOWER_HSV = 60
UPPER_HSV = 100
stop_pipes = []

if __name__ == "__main__":
    # create a series of pipes over which we can send the
    # shut down signal; pipes are one-way with an in and
    # and out ends (here called writer and reader, respectively)
    cs_reader, cs_writer = Pipe(duplex=False)
    nt_reader, nt_writer = Pipe(duplex=False)
    trgt_reader, trgt_writer = Pipe(duplex=False)
    stop_pipes.append(cs_writer)
    stop_pipes.append(nt_writer)
    stop_pipes.append(trgt_writer)
    # Create a queues for 2-way comm between the targeting
    # and camstreamer queues
    targeting_queue = Queue()

    camstreamer_process = Camstreamer(targeting_queue=targeting_queue, stop_pipe=cs_reader)
    target_process = Targeting(targeting_queue=targeting_queue,
                               stop_pipe=trgt_reader,
                               source=CAMERA_SOURCE,
                               lower_hsv=LOWER_HSV,
                               upper_hsv=UPPER_HSV)

    camstreamer_process.start()
    target_process.start()
    while True:
        # wait for Esc or q key and then exit
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            for pipe in stop_pipes:
                pipe.put("stop")
            time.sleep(2)
            camstreamer_process.terminate()
            target_process.terminate()
            exit()
