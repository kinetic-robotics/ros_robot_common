#!/usr/bin/env python3
import time, argparse, libusb, math
import matplotlib.pyplot as plt
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
    count = 1
    canID = 0x2FF << 1
    testCount = 100
    oneCountLength = 5
    speedList = []
    xAxis = range(1, testCount + 1)
    sendData = []
    while len(sendData) / 16 < oneCountLength: 
        cmdID = 10 + count % 2
        count = count + 1
        sendData += [0xAA, cmdID, 12]
        sendData += [0xAA ^ cmdID ^ 12]
        sendData += canID.to_bytes(4, "big")
        sendData += count.to_bytes(8, "big")
    handle = initUSBDevice()
    # 清除缓存数据
    for x in range(10):
        (recvData, readTime) = readUSB(handle, len(sendData))
    while len(speedList) < testCount:
        count = count + 1
        startTime = time.time()
        for x in range(math.ceil(len(sendData) / 2047)):
            writeUSB(handle, sendData[2046 * x : 2046 * (x + 1)])
        recvData = []
        packetCount = 0
        while True:
            (tmpRecvData, readTime) = readUSB(handle, len(sendData))
            recvData += tmpRecvData
            packetCount = packetCount + 1
            if len(recvData) == len(sendData):
                break
        stopTime = time.time()
        speedList.append(len(sendData) * 2 * 1 / (stopTime - startTime) / 1000)
        print("Testing....", str(round(len(speedList) / testCount * 100, 1)) + "%")
    plt.title('USB CAN Loopback Speed Test')
    plt.xlabel('Time')
    plt.ylabel('Bidirectional(Tx + Rx) Speed (kByte/s)')
    plt.plot(xAxis, speedList)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test usb serial speed and plot.')
    args = parser.parse_args()
    main(args)