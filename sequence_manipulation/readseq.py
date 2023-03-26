import struct

with open('E:/sequence.bin', 'rb') as f:
    prog_size, led_count = struct.unpack('ii', f.read(8))

    commands = [
        struct.unpack('B'*(3*led_count+2)+'H', kek:=f.read(3*led_count+4)) for _ in range(prog_size)
    ]

print(commands)