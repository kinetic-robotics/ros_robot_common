#!/usr/bin/env python3
import time, argparse, libusb
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
    writeData = (ct.c_uint8 * 256)(*data)
    startTime = time.time()
    libusb.bulk_transfer(handle, ct.c_uint8(0x01), ct.cast(writeData, ct.POINTER(ct.c_uint8)), ct.c_int(len(data)), ct.pointer(readLength), ct.c_uint(1))
    stopTime = time.time()
    return (stopTime - startTime) * 1000

def readUSB(handle, length):
    buffer = (ct.c_ubyte * length)()
    readLength = ct.c_int()
    startTime = time.time()
    libusb.bulk_transfer(handle, ct.c_uint8(0x81), ct.cast(buffer, ct.POINTER(ct.c_uint8)), ct.c_int(length), ct.pointer(readLength), ct.c_uint(1))
    stopTime = time.time()
    buffer = list(buffer)
    return (buffer[:readLength.value], (stopTime - startTime) * 1000)

def main(args):
    count = 10
    canID = 2
    listLength = 100000
    delayList = []
    warningList = []
    xAxis = range(1, listLength + 1)
    handle = initUSBDevice()
    startScriptTime = time.time()
    while len(delayList) < listLength:
        count = count + 1
        sendData = [0xAA, 10, 12, 0 ]
        sendData[3] = sendData[0] ^ sendData[1] ^ sendData[2]
        sendData += canID.to_bytes(4, "big")
        sendData += count.to_bytes(8, "big")
        startTime = time.time()
        writeTime = writeUSB(handle, sendData)
        (recvData, readTime) = readUSB(handle, 16)
        stopTime = time.time()
        if len(recvData) != 16:
            print("Read timeout!")
            return
        if recvData[0] != 0xAA or recvData[1] != 1 or recvData[2] != 12 or recvData[3] != (recvData[0] ^ recvData[1] ^ recvData[2]) or recvData[4:] != sendData[4:]:
            print("Read data error!")
            return
        delay = (stopTime - startTime) * 1000
        delayList.append(delay)
        if delay > 1:
            warningList.append("Detected very large delay! Delay = {}ms, time = {}s, write time = {}ms, read time = {}ms.".format(delay, startTime - startScriptTime, writeTime, readTime))
        print("Testing....", str(round(len(delayList) / listLength * 100, 1)) + "%")
    for output in warningList:
        print(output)
    plt.title('USB CAN Loopback Delay Test')
    plt.xlabel('Time')
    plt.ylabel('Bidirectional Delay (ms)')
    plt.plot(xAxis,delayList)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test usb serial delay and plot.')
    args = parser.parse_args()
    main(args)