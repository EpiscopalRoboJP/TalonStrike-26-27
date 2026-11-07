"""Camera, image, and video inputs for the laptop pollen proxy."""

import os
import time

import cv2

REQUEST_WIDTH = 640
REQUEST_HEIGHT = 480
BUILTIN_HINTS = (
    "integrated",
    "built-in",
    "builtin",
    "facetime",
    "truevision",
    "easycamera",
    "easy camera",
    "ir camera",
    "infrared",
)


class CameraSource:
    def __init__(self, index, name=None, backend=None):
        self.cap = None
        self.index = index
        self.name = name or "camera {0}".format(index)
        self.backend = backend if backend is not None else (
            cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        )
        self._open()

    def reopen(self, index, name, backend):
        self.close()
        self.index = index
        self.name = name
        self.backend = backend
        self._open()

    def _open(self):
        self.cap = cv2.VideoCapture(self.index, self.backend)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_HEIGHT)
        if not self.cap.isOpened() or self._first_frame() is None:
            self.cap.release()
            self.cap = None
            raise RuntimeError(
                "{0} is connected, but it is not sending video. "
                "Unplug it, wait a few seconds, and plug it into another USB port.".format(self.name)
            )

    def _first_frame(self):
        for _attempt in range(30):
            ok, frame = self.cap.read()
            if ok and frame is not None:
                return frame
            time.sleep(0.05)
        return None

    def read(self):
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class ImageSource:
    def __init__(self, path):
        frame = cv2.imread(path)
        if frame is None:
            raise RuntimeError("Could not read image " + path)
        self.frame = frame

    def read(self):
        return self.frame.copy()

    def close(self):
        return None


class VideoSource:
    def __init__(self, path):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError("Could not read video " + path)

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


def list_cameras():
    """DirectShow cameras as OpenCV sees them, laptop webcam included."""
    try:
        from cv2_enumerate_cameras import enumerate_cameras
    except ImportError:
        return []
    api = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    cameras = []
    for camera in enumerate_cameras(api):
        cameras.append({
            "index": int(camera.index),
            "name": camera.name,
            "backend": int(camera.backend),
        })
    return cameras


def is_builtin_camera(name):
    lowered = name.lower()
    return any(hint in lowered for hint in BUILTIN_HINTS)


def is_virtual_camera(name):
    lowered = name.lower()
    return "virtual" in lowered or "obs" in lowered


def choose_camera(cameras, requested_index):
    """Prefer an ELP, then any camera that is not the laptop webcam."""
    if requested_index is not None:
        for camera in cameras:
            if camera["index"] == requested_index:
                return camera
        return {
            "index": requested_index,
            "name": "camera {0}".format(requested_index),
            "backend": cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY,
        }
    for camera in cameras:
        if "elp" in camera["name"].lower():
            return camera
    physical = [camera for camera in cameras if not is_virtual_camera(camera["name"])]
    external = [camera for camera in physical if not is_builtin_camera(camera["name"])]
    if external:
        return external[0]
    if physical:
        return physical[0]
    if cameras:
        return cameras[0]
    return {
        "index": 0,
        "name": "camera 0",
        "backend": cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY,
    }


def open_windows_camera(camera):
    try:
        from vision.winrt_camera import WinRtCamera
    except ImportError:
        from winrt_camera import WinRtCamera
    return WinRtCamera(camera["name"], camera["index"], camera["backend"])


def open_capture(camera):
    """The ELP streams for the Windows Camera app, not for OpenCV."""
    external = os.name == "nt" and not is_builtin_camera(camera["name"]) and not is_virtual_camera(camera["name"])
    windows_error = None
    if external:
        try:
            return open_windows_camera(camera)
        except (RuntimeError, ImportError) as error:
            windows_error = error
    try:
        return CameraSource(camera["index"], camera["name"], camera["backend"])
    except RuntimeError as opencv_error:
        if windows_error is not None:
            raise RuntimeError("{0} {1}".format(windows_error, opencv_error))
        raise


class CaptureSource:
    """One live camera. reopen() can switch between OpenCV and the Windows API."""

    def __init__(self, camera):
        self.inner = None
        self.notice = None
        self._open(camera)

    def _open(self, camera):
        self.inner = open_capture(camera)
        self.index = camera["index"]
        self.name = self.inner.name
        self.backend = camera.get("backend")

    def read(self):
        if self.inner is None:
            return None
        return self.inner.read()

    def close(self):
        if self.inner is not None:
            self.inner.close()
            self.inner = None

    def reopen(self, index, name, backend):
        self.close()
        self._open({"index": index, "name": name, "backend": backend})


def build_source(args):
    if args.image:
        return ImageSource(args.image), []
    if args.video:
        return VideoSource(args.video), []
    cameras = list_cameras()
    chosen = choose_camera(cameras, args.camera)
    if not cameras:
        cameras = [chosen]
    rest = [
        camera for camera in cameras
        if camera["index"] != chosen["index"] or camera["backend"] != chosen["backend"]
    ]
    order = [chosen] + [camera for camera in rest if not is_virtual_camera(camera["name"])]
    order += [camera for camera in rest if is_virtual_camera(camera["name"])]
    failure = None
    for camera in order:
        try:
            source = CaptureSource(camera)
        except RuntimeError as error:
            failure = str(error)
            print(failure)
            continue
        if failure is not None:
            source.notice = failure + " Showing {0} instead. Press c to try again.".format(source.name)
        return source, cameras
    raise RuntimeError(failure or "Could not open a camera.")
