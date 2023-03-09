import struct

with open('D:/sequence.bin', 'rb') as f:
    prog_size, led_count, fps = struct.unpack('iiB', f.read(9))

    commands = [
        struct.unpack('B'*(3*led_count+2), f.read(3*led_count+2)) for _ in range(prog_size)
    ]

print(commands)