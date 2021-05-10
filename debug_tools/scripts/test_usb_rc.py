#!/usr/bin/env python3
import time, argparse, libusb
import ctypes as ct

def initUSBDevice():
    if libusb.init(None) < 0:
        print("failed to init libusb!")
        return
    # 获取设备
    devs = ct.POINTER(ct.POINTER(libusb.device))()
    count = libusb.get_device_list(None, devs)
    if count < 0:
        print("failed to get device list!")
        return
    targetDev = None
    for dev in devs:
        desc = libusb.device_descriptor()
        if libusb.get_device_descriptor(dev, ct.byref(desc)) < 0:
            print("failed to get device descriptor.")
            return
        if desc.idVendor == 0x2413 and desc.idProduct == 0x0c4c:
            print("Found Device!")
            targetDev = dev
            break

    handle = ct.POINTER(libusb.device_handle)()
    # 打开设备和自动挂载
    if libusb.open(targetDev, ct.byref(handle)) != libusb.LIBUSB_SUCCESS or libusb.set_auto_detach_kernel_driver(handle, 1):
        return None
    # 注册接口
    if libusb.claim_interface(handle, 1) != libusb.LIBUSB_SUCCESS:
        return None
    return handle

def writeUSB(handle, data):
    readLength = ct.c_int()
    writeData = (ct.c_uint8 * len(data))(*data)
    startTime = time.time()
    libusb.bulk_transfer(handle, ct.c_uint8(0x01), ct.cast(writeData, ct.POINTER(ct.c_uint8)), ct.c_int(len(data)), ct.pointer(readLength), ct.c_uint(100))
    stopTime = time.time()
    return (stopTime - startTime) * 1000

def readUSB(handle, length):
    buffer = (ct.c_ubyte * length)()
    readLength = ct.c_int()
    startTime = time.time()
    libusb.bulk_transfer(handle, ct.c_uint8(0x81), ct.cast(buffer, ct.POINTER(ct.c_uint8)), ct.c_int(length), ct.pointer(readLength), ct.c_uint(100))
    stopTime = time.time()
    buffer = list(buffer)
    return (buffer[:readLength.value], (stopTime - startTime) * 1000)

def main(args):
    handle = initUSBDevice()
    machine = 0
    cmdID = 0
    totalLength = 0
    length = 0
    recvLength = 0
    seq = 0
    while True:
        (recvData, readTime) = readUSB(handle, 1024)
        for byte in recvData:
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
                        seq = seq + 1
                        print(seq)
                        totalLength = 0
                if recvLength == length:
                    machine = 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test usb rc.')
    parser.add_argument('--path', help="USB serial path", default="/dev/ttyACM0")
    args = parser.parse_args()
    main(args)