import numpy as np
from numpy import uint8

from sequence_manipulation.tzeentch.program import CubeProgram

prog = CubeProgram(cube_size=6, target_fps=20)
Frame = prog.frame_factory()


for coord in range(prog.cube_size):
    data = np.zeros([prog.cube_size, prog.cube_size, prog.cube_size, 3], dtype=uint8)
    data[0, 0, coord] = [255, 128, 64]
    data[0, coord, 0] = [255, 128, 64]
    data[coord, 0, 0] = [255, 128, 64]
    prog.add_frame(
        Frame(
            interpolation_steps=10,
            frame_data=data
        )
    )

for coord in range(prog.cube_size):
    data = np.zeros([prog.cube_size, prog.cube_size, prog.cube_size, 3], dtype=uint8)
    data[0, prog.cube_size-1, coord] = [255, 128, 64]
    data[prog.cube_size-1, 0, coord] = [255, 128, 64]
    data[prog.cube_size-1, coord, 0] = [255, 128, 64]
    data[0, coord, prog.cube_size-1] = [255, 128, 64]
    data[coord, 0, prog.cube_size-1] = [255, 128, 64]
    data[coord, prog.cube_size-1, 0] = [255, 128, 64]
    prog.add_frame(
        Frame(
            interpolation_steps=10,
            frame_data=data
        )
    )

for coord in range(prog.cube_size):
    data = np.zeros([prog.cube_size, prog.cube_size, prog.cube_size, 3], dtype=uint8)
    data[prog.cube_size-1, prog.cube_size-1, coord] = [255, 128, 64]
    data[prog.cube_size-1, coord, prog.cube_size-1] = [255, 128, 64]
    data[coord, prog.cube_size-1, prog.cube_size-1] = [255, 128, 64]
    prog.add_frame(
        Frame(
            interpolation_steps=10,
            frame_data=data
        )
    )

data = prog.serialize()

with open('E:/running_frame.bin', 'wb') as f:
    prog.serializes(f)

