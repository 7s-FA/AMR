"""Display annotated frames from the robot. No detection or control publication."""
import time
import cv2
from .source import RosSource


def main():
    source = RosSource('/burger2/docking/annotated/compressed', compressed=True)
    title = 'burger2 robot ArUco result (q: close viewer only)'
    waiting = True
    window_open = False
    try:
        print('Waiting for robot-local docking vision. Closing this window does not stop the robot.', flush=True)
        while source.rclpy.ok():
            try:
                frame, metadata = source.read(.2)
            except RuntimeError:
                if not waiting:
                    cv2.putText(frame, 'STREAM LOST - robot status is separate', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
                    cv2.imshow(title, frame)
                    waiting = True
            else:
                stamp = metadata['source_stamp']
                age = time.time()-stamp['sec']-stamp['nanosec']/1e9
                cv2.putText(frame, f'ROBOT DETECTION | display age {age:.2f}s', (10, frame.shape[0]-12),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
                cv2.imshow(title, frame)
                window_open = True
                waiting = False
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
            if window_open and cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
