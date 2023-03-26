from __future__ import annotations

import struct
import sys
from functools import partial, partialmethod
from io import BytesIO, BufferedIOBase
from typing import List, Type, get_type_hints, TypeVar, Generic, get_args, get_type_hints, BinaryIO

from sequence_manipulation.tzeentch.frame import Frame

FT = TypeVar('FT', bound=Frame)


class CubeProgram(Generic[FT]):
    frames: List[FT] = None
    cube_size: int
    target_fps: int

    def __init__(self, cube_size: int, target_fps: int = 60, frames: List[FT] = None):
        self.cube_size = cube_size
        self.target_fps = target_fps
        if self.frames is not None:
            self.frames = frames
        else:
            self.frames = []

    def add_frame(self, frame: FT) -> CubeProgram:
        self.frames.append(frame)
        return self

    def serializes(self, dest: BinaryIO) -> int:
        bytes_written = dest.write(struct.pack('ii', len(self.frames), self.cube_size ** 3))
        for frame in self.frames:
            bytes_written += frame.serialize(dest)
        return bytes_written

    def serialize(self) -> bytes:
        buff = BytesIO()
        self.serializes(buff)
        return buff.getvalue()

    def frame_factory(self) -> Type[FT]:
        try:
            frame_type = get_args(self.__orig_class__)[0]
        except AttributeError:
            frame_type = Frame

        class PartialFrame(frame_type):
            __init__ = partialmethod(frame_type.__init__, cube_size=self.cube_size, time_ms=1000//self.target_fps)
        return PartialFrame
