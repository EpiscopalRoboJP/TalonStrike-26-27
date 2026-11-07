"""Read the ELP through the same Windows camera API the Camera app uses.

OpenCV can see this camera and still get no frames from it. The Windows
frame reader is the path that actually delivers video.
"""

import asyncio
import threading

import cv2
import numpy as np
from winsdk.windows.graphics.imaging import BitmapPixelFormat, SoftwareBitmap
from winsdk.windows.media.capture import (
    MediaCapture,
    MediaCaptureInitializationSettings,
    MediaCaptureMemoryPreference,
    MediaCaptureSharingMode,
    StreamingCaptureMode,
)
from winsdk.windows.media.capture.frames import MediaFrameSourceGroup, MediaFrameSourceKind
from winsdk.windows.storage.streams import Buffer, DataReader


class WinRtCamera:
    def __init__(self, name, index=None, backend=None):
        self.name = name
        self.index = index
        self.backend = backend
        self._frame = None
        self._error = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        if not self._ready.wait(8):
            self._error = self._error or "{0} did not send a frame.".format(name)
        if self._error:
            self.close()
            raise RuntimeError(self._error)

    def read(self):
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    def close(self):
        self._stop.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=3)

    def _thread_main(self):
        try:
            asyncio.run(self._run())
        except Exception as error:
            self._error = "{0}: {1}".format(self.name, error)
        finally:
            self._ready.set()

    async def _run(self):
        groups = await MediaFrameSourceGroup.find_all_async()
        group = next((item for item in groups if item.display_name == self.name), None)
        if group is None:
            self._error = "Windows camera list has no {0}.".format(self.name)
            return

        settings = MediaCaptureInitializationSettings()
        settings.source_group = group
        settings.sharing_mode = MediaCaptureSharingMode.SHARED_READ_ONLY
        settings.streaming_capture_mode = StreamingCaptureMode.VIDEO
        settings.memory_preference = MediaCaptureMemoryPreference.CPU
        capture = MediaCapture()
        await capture.initialize_async(settings)
        info = next(
            (item for item in group.source_infos if item.source_kind == MediaFrameSourceKind.COLOR),
            None,
        )
        if info is None:
            capture.close()
            self._error = "{0} has no color stream.".format(self.name)
            return

        source = capture.frame_sources[info.id]
        reader = await capture.create_frame_reader_async(source)
        started = await reader.start_async()
        if int(started) != 0:
            capture.close()
            self._error = "{0} did not start (status {1}).".format(self.name, int(started))
            return

        try:
            while not self._stop.is_set():
                frame = reader.try_acquire_latest_frame()
                if frame is None:
                    await asyncio.sleep(0.01)
                    continue
                try:
                    video = frame.video_media_frame
                    bitmap = None if video is None else video.software_bitmap
                    image = None if bitmap is None else bitmap_to_bgr(bitmap)
                finally:
                    frame.close()
                if image is None:
                    continue
                with self._lock:
                    self._frame = image
                self._ready.set()
        finally:
            await reader.stop_async()
            capture.close()


def bitmap_to_bgr(bitmap):
    converted = SoftwareBitmap.convert(bitmap, BitmapPixelFormat.BGRA8)
    try:
        width = converted.pixel_width
        height = converted.pixel_height
        if width < 2 or height < 2:
            return None
        buffer = Buffer(width * height * 4)
        converted.copy_to_buffer(buffer)
        raw = bytearray(buffer.length)
        DataReader.from_buffer(buffer).read_bytes(raw)
        bgra = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    finally:
        if converted is not bitmap:
            converted.close()
