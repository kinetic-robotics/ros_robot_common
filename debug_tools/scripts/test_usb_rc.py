#!/usr/bin/env python3
from __future__ import print_function
import serial, time, argparse
import matplotlib.pyplot as plt

def main(args):
    with serial.Serial(args.path, timeout=1) as ser:
        machine = 0
        cmdID = 0
        totalLength = 0
        length = 0
        recvLength = 0
        while True:
            byte = ser.read(1)
            if byte == "":
                continue
            byte = int(byte.encode('hex'), 16)
            if machine == 0:
                if byte == 0xAA:
                    machine = machine + 1
            elif machine == 1:
                cmdID = byte
                machine = machine + 1
            elif machine == 2:
                length = byte
                recvLength = 0
                machine = machine + 1
            elif machine == 3:
                if 0xAA ^ cmdID ^ length == byte:
                    machine = machine + 1
                else:
                    machine = 0
            elif machine == 4:
                recvLength = recvLength + 1
                if cmdID == 3:
                    print(byte, end=' ')
                    totalLength = totalLength + 1
                    if totalLength == 18:
                        print("")
                        totalLength = 0
                if recvLength == length:
                    machine = 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test usb rc.')
    parser.add_argument('--path', help="USB serial path", default="/dev/ttyACM0")
    args = parser.parse_args()
    main(args)