import struct

import numpy as np
from numpy import sin, cos, vstack, pi, uint8, ones

PROG_LEN = 50
LED_COUNT = 6**3
DESIRED_FPS = 20

with open('D:/sequence.bin', 'wb') as f:
    f.write(struct.pack('iiB', PROG_LEN, LED_COUNT, DESIRED_FPS))

    for x in range(PROG_LEN):

        shift = x/PROG_LEN*2*pi
        space = np.linspace(0 + shift, 2*pi + shift, LED_COUNT)

        f.write(struct.pack(
            'B'*(3*LED_COUNT+2),
            *vstack([
                ((sin(space)+1) * 127).astype(dtype=uint8),
                ((cos(space)+1) * 127).astype(dtype=uint8),
                ones(LED_COUNT, dtype=uint8) * 32]).T.flatten(),
            1, 10
            ),
        )


