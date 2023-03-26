import struct

import numpy as np
from numpy import sin, cos, vstack, pi, uint8, ones, zeros

PROG_LEN = 2
LED_COUNT = 6**3
DESIRED_FPS = 20

with open('E:/sequence_simple.bin', 'wb') as f:
    f.write(struct.pack('ii', PROG_LEN, LED_COUNT))

    for x in range(PROG_LEN):

        shift = x/PROG_LEN*2*pi
        space = np.linspace(0 + shift, 2*pi + shift, LED_COUNT)

        f.write(struct.pack(
            'B'*(3*LED_COUNT+2)+'H',
            *vstack([
                zeros(LED_COUNT, dtype=uint8),
                zeros(LED_COUNT, dtype=uint8),
                ones(LED_COUNT, dtype=uint8) * 128]).T.flatten(),
            0, 10, 50
        ),
    )
