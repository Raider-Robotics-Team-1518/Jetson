import cv2
import robovision as rv
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--source", required=False,
                    help="Video source, e.g. the webcam number")
    args = vars(ap.parse_args())
    source = args["source"]
    vs = None
    try:
        webcam = int(source)
        vs = rv.VideoStream(source="webcam", cam_id=webcam)
    except ValueError:
        if source == "picam":
            vs = rv.VideoStream(source="picam")
        else:
            vs = rv.VideoStream(source="ipcam", ipcam_url=source)
    if source is None:
        print("Invalid source")
        exit()
    vs.start()
    params = rv.load_camera_params('params.pickle')
    cv2.namedWindow('camera', cv2.WINDOW_NORMAL)
    while True:
        frame = vs.read_frame()
        frame = rv.flatten(frame, params)
        cv2.imshow('camera', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            vs.stop()
            break


class Object(object):
    """
    An empty, generic object constructor required for de-pickling
    the camera parameters file
    """
    pass

if __name__ == "__main__":
    main()
