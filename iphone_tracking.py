'''
iPhone tracking: reads a live MJPEG video stream from an iPhone (via the IP
Camera Lite app) instead of a local webcam, and draws the same AprilTag
(tag36h11, ID 14) detection overlay autopark.py does.

This is the reverse physical setup from autopark.py: there, the camera is
stationary and the tag moves with the car. Here, the tag is fixed somewhere
in the room and the camera (the iPhone) is the thing that moves. That means
there's no "centered in frame" target and no parking logic - this script is
2D tracking only: find the tag in each frame and show where it is. No BLE,
no motor control.

Usage:
    python iphone_tracking.py

You'll be prompted for the MJPEG stream URL IP Camera Lite lists on its
screen, e.g. http://192.168.1.42:8080/video - use the MJPEG link it shows,
not the plain http://<ip>:8080/ landing page.

'''
import cv2
import numpy as np

TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TARGET_TAG_ID = 14

MAX_CONSECUTIVE_READ_FAILURES = 30     # ~1s of dropped frames over Wi-Fi before giving up


def tag_metrics(corners):
    """corners: (4,2) array, detector order. Returns (centroid_xy, side_length_px)."""
    centroid = corners.mean(axis=0)
    side_px = np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)])
    return centroid, side_px


stream_url = input("iPhone stream URL (e.g. rtsp://192.168.1.42:8554/live): ").strip()
cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)

aruco_dict = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

consecutive_failures = 0

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            # A WiFi stream drops frames now and then; a wired webcam (autopark.py's
            # case) doesn't, so that script treats a failed read as fatal. Here we
            # tolerate brief drops and only give up if they don't stop.
            consecutive_failures += 1
            if consecutive_failures > MAX_CONSECUTIVE_READ_FAILURES:
                print("Lost the stream - too many consecutive failed reads.")
                break
            continue
        consecutive_failures = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = detector.detectMarkers(gray)

        target_corners = None
        if ids is not None:
            for c, tag_id in zip(corners_list, ids.flatten()):
                if tag_id == TARGET_TAG_ID:
                    target_corners = c.reshape(4, 2)
                    break

        if target_corners is not None:
            centroid, side_px = tag_metrics(target_corners)

            cv2.polylines(frame, [target_corners.astype(int)], isClosed=True,
                          color=(0, 0, 255), thickness=2)
            cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"({centroid[0]:.0f}, {centroid[1]:.0f})  size={side_px:.0f}px",
                        (int(centroid[0]) + 10, int(centroid[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        status = "TRACKING" if target_corners is not None else "NO TAG"
        cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        cv2.imshow("iPhone tracking - press q to quit", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    print("Stopped.")
