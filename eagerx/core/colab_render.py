# import dependencies
try:
    from google.colab.output import eval_js
    from IPython.display import display, Javascript
except ImportError:
    Warning(
        "It seems that you are trying to use this package outside of  google colab. Using a mock version instead, "
        "that does not do anything."
    )

    def mock_fn(*args, **kwargs):
        pass

    eval_js = mock_fn
    display = mock_fn
    Javascript = mock_fn

from base64 import b64encode
import numpy as np
from collections import deque
from typing import List, Union, Optional
import PIL
import io
import threading
import time


class InlineRender:
    def __init__(self, fps: int, maxlen: int = 200, shape: Optional[List[int]] = None, maxwidth: int = 300):
        self.fps = fps
        self.buffer = deque(maxlen=maxlen)
        self.maxlen = maxlen
        self.shape = shape if isinstance(shape, list) else [28, 28]
        self.timestamp = time.time()
        interval = max(1, int(1000 / fps))
        js = Javascript(
            """
        const queue = []
        // Frame
        const div = document.createElement('div');
        div.style.border = '2px solid black';
        div.style.padding = '3px';
        div.style.width = '100%';
        div.style.maxWidth = 'MAXWIDTHpx';
        document.body.appendChild(div);
        // Video
        const img = document.createElement('img');
        img.id = 'my_video'
        img.style.display = 'block';
        img.width = div.clientWidth - 6;
        div.appendChild(img)
        setInterval(() => {
          const n_img = queue.pop()
          if (n_img) {
            img.src = n_img;
          }
        }, 1/FPS) // 1/FPS is replaced with interval argument
        // Synchronize button
        const capture = document.createElement('button');
        capture.textContent = 'Synchronize';
        div.appendChild(capture);
        capture.onclick = function(){
          queue.length= 0
          };
        // Add frames to buffer
        function push_images(...images) {
          // images.forEach(image => queue.push(image))
          images.forEach(image => queue.unshift(image))
        }
    """.replace(
                "1/FPS", str(interval)
            ).replace(
                "MAXWIDTH", str(maxwidth)
            )
        )
        display(js)

        # Create async push loop
        self.event = threading.Event()
        thread = threading.Thread(target=self._async_pushing, args=())
        thread.start()

    def _async_pushing(self):
        """Asynchronously push images from client (server that hosts Python) to local browser (js)."""
        while True:
            self.event.wait()  # Wait for event (ie new buffered images)
            self.event.clear()  # Clear event
            self._push()  # Push buffered images

    def _push(self):
        """Push buffered images from client (server that hosts Python) to local browser (js)."""
        # print("pushing")
        msg = "push_images("
        for _ in range(len(self.buffer)):
            img = self.buffer.popleft()
            msg += '"{}",'.format(img)
        eval_js(msg[:-1] + ")", ignore_result=True, timeout_sec=1.0)
        # print("finished")

    def _img_to_bytes(self, img: np.ndarray):
        """Convert image to JS format

        :params bbox_array: Numpy array (pixels) containing rectangle to overlay on video stream.
        :returns: Base64 image byte string.
        """
        # convert array into PIL image
        img_PIL = PIL.Image.fromarray(img).convert("RGB")
        # resize image if not matching desired self.shape (defined in .yaml)
        if img.shape[:2] != tuple(self.shape):
            img_PIL.thumbnail(self.shape, PIL.Image.ANTIALIAS)
        # format bbox into png for return
        iobuf = io.BytesIO()
        img_PIL.save(iobuf, format="png")
        # format return string
        img_bytes = "data:image/png;base64,{}".format((str(b64encode(iobuf.getvalue()), "utf-8")))
        return img_bytes

    def buffer_images(self, images: Union[List[np.ndarray], np.ndarray]):
        """Buffer images at client side (server that hosts Python)."""
        self.timestamp = time.time()
        if not isinstance(images, list):
            images = [images]
        # Add images to buffer
        for img in images:
            self.buffer.append(self._img_to_bytes(img))
        # Signal push thread that new images are buffered
        self.event.set()
        # Check for buffer overflow
        if len(self.buffer) == self.maxlen:
            return False  # Buffer about to overflow
        else:
            return True  # Buffer not full.
