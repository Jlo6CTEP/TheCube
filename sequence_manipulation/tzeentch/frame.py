import struct
from enum import Enum, auto, IntEnum
from io import BufferedIOBase, BytesIO
from typing import BinaryIO

import numpy
import numpy as np
from numpy import uint8, reshape
from numpy._typing import NDArray


class Rotation(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()


class Frame:
    """

    First index - layer where 0 is the bottom layer
    Second index - width
    Third index - depth
    Forth - color

    This assumes that leds are arranged as follows
        * Leds are represented with *
        * Wires are represented with - and |
        * Arrow-like (<>^V) symbols represent DINs and DOUTs of a LED line
          Arrow pointing away from the LED is DOUT
          Arrow pointing towards the LED is DIN

    layer 0                layer 1                ^  layer 3     v       layer 4
    *----*    *----*       *----*----*----* >     *    *----*    *     > *----*----*----*
    |    |    |    |       |                      |    |    |    |                      |
    *    *    *    *       *----*----*----*       *    *    *    *       *----*----*----*
    |    |    |    |                      |       |    |    |    |       |
    *    *    *    *       *----*----*----*       *    *    *    *       *----*----*----*
    |    |    |    |       |                      |    |    |    |                      |
    *    *----*    *       *----*----*----*       *----*    *----*       *----*----*----*
    ^              v                      ^                              V
    """
    buffer: NDArray[uint8]
    cube_size: int
    interpolation_steps: bool
    time_ms: int

    def __init__(self, cube_size: int, time_ms: int, interpolation_steps: bool = False,
                 frame_data: NDArray[uint8] = None):
        self.cube_size = cube_size
        self.interpolation_steps = interpolation_steps
        self.buffer = frame_data.astype(uint8)
        self.time_ms = time_ms

    def serializes(self) -> bytes:
        buff = BytesIO()
        self.serialize(buff)
        return buff.getvalue()

    def serialize(self, dest: BinaryIO) -> int:
        rotation: int = Rotation.ZERO
        bytes_written = 0
        for layer in self.buffer:
            match rotation:
                case Rotation.ZERO:
                    for line in range(self.cube_size):
                        bytes_written += dest.write(
                            reshape((layer[:, line])[::(1 if line % 2 == 0 else -1)], -1).tobytes())
                case Rotation.ONE:
                    for line in range(self.cube_size):
                        bytes_written += dest.write(
                            reshape((layer[line, :])[::(-1 if line % 2 == 0 else 1)], -1).tobytes())
                case Rotation.TWO:
                    for line in range(self.cube_size):
                        bytes_written += dest.write(
                            reshape((layer[:, -line-1])[::(-1 if line % 2 == 0 else 1)], -1).tobytes())
                case Rotation.THREE:
                    for line in range(self.cube_size):
                        bytes_written += dest.write(
                            reshape((layer[-line-1, :])[::(1 if line % 2 == 0 else -1)], -1).tobytes())
            rotation = Rotation((rotation + 1) % len(Rotation))

        footer = struct.pack('BBH', *[
            1 if self.interpolation_steps != 0 else 0, self.interpolation_steps, self.time_ms
        ])
        bytes_written += dest.write(footer)
        return bytes_written


