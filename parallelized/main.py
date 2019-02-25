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
import imutils
import time
from multiprocessing import Process, Queue


class CamGrabber(Process):
    def __init__(self, queue=None, stop_queue=None, **kwargs):
        super(CamGrabber, self).__init__()
        self.queue = queue
        self.stop_queue = stop_queue
        self.kwargs = kwargs

    def run(self):
        cam = cv2.VideoCapture(0)
        keepGoing = True
        while keepGoing:
            success, frame = cam.read()
            if success:
                frame = imutils.resize(frame, width=320)
                self.queue.put(frame)
            else:
                print("frame fail {}".format(success))
            if self.stop_queue.empty() is False:
                stop = self.stop_queue.get()
                if stop == 1:
                    print("CamGrabber exiting")
                    cam.release()
                    keepGoing = False
                    break


class FrameProcessor(Process):
    def __init__(self, queue=None, stop_queue=None, **kwargs):
        super(FrameProcessor, self).__init__()
        self.queue = queue
        self.stop_queue = stop_queue
        self.kwargs = kwargs

    def run(self):
        keepGoing = True
        while keepGoing:
            if self.queue.empty() is False:
                f = self.queue.get()
                if f is not None:
                    adjusted = cv2.addWeighted(f,
                                               1. + float(40) / 127.,
                                               f,
                                               float(1),
                                               float(60) - float(40))

                    cv2.imshow("Adjusted", adjusted)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("FrameProcessor exiting")
                self.stop_queue.put(1)
                cv2.destroyAllWindows()
                keepGoing = False
                break


if __name__ == "__main__":
    q = Queue()
    cg_stop_queue = Queue()
    stop_queue = Queue()
    cg = CamGrabber(queue=q, stop_queue=cg_stop_queue)
    fp = FrameProcessor(queue=q, stop_queue=stop_queue)
    cg.start()
    fp.start()
    while True:
        if stop_queue.empty() is False:
            stop = stop_queue.get()
            if stop == 1:
                print("Signaling CamGrabber to exit")
                cg.stop_queue.put(1)
                fp.terminate()
                time.sleep(2)
                cg.terminate()
                exit()
