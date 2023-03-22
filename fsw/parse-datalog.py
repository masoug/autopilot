import struct
import sys


FMT_STR = '<fffffffffffH'
FRAME_LEN = struct.calcsize(FMT_STR)


def main(args):
    fp = open(args[1], 'rb')

    buf = fp.read(FRAME_LEN)
    while len(buf) == FRAME_LEN:
        frame = struct.unpack(FMT_STR, buf)
        print(','.join(map(str, frame)))
        buf = fp.read(FRAME_LEN)

    fp.close()

if __name__ == '__main__':
    exit(main(sys.argv))
