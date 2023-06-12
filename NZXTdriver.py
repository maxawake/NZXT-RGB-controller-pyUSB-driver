#!/usr/bin/python
import usb.core
import usb.util

FAN_MAP = [0x01, 0x02, 0x04]

COLOR_MODES = {
    "fixed":            (0x00, 0x32, 0x00, 0x01, 0x00),
    "spectrum-wave":    (0x02, 0xfa, 0x00, 0x01, 0x00),
    "breathing":        (0x07, 0x14, 0x00, 0x01, 0x08),
    "fading":           (0x01, 0x28, 0x00, 0x03, 0x08),
    "pulse":            (0x06, 0x0f, 0x00, 0x01, 0x08),
    "alternating":      (0x05, 0xe8, 0x03, 0x02, 0x00),
    "rainbow-pulse":    (0x0d, 0xfa, 0x00, 0x01, 0x00),
    "super-rainbow":    (0x0c, 0xfa, 0x00, 0x01, 0x00),
    "rainbow-flow":     (0x0b, 0xfa, 0x00, 0x01, 0x00),
    "starry-night":     (0x09, 0x0f, 0x00, 0x01, 0x00)
}

def rgb2hex(r, g, b):
    r_hex = eval("0x{:02x}".format(r))
    g_hex = eval("0x{:02x}".format(g))
    b_hex = eval("0x{:02x}".format(b))
    return r_hex, g_hex, b_hex

class NZXTUsbDriver:
    def __init__(self, verbose=False):
        self.verbose = verbose

        self.dev = usb.core.find(idVendor=0x1e71, idProduct=0x2012)

        try:
            self.dev.reset()
        except Exception as e:
            print("reset", e)

        if self.dev.is_kernel_driver_active(0):
            if self.verbose:
                print("Detaching kernel driver")
            self.dev.detach_kernel_driver(0)

        self.endpoint_in = self.dev[0][(0, 0)][0]
        self.endpoint_out = self.dev[0][(0, 0)][1]

        self.colors = [0x00 for i in range(8*3)]

    def send_data(self, inBuffer, receive=False):
        # Send a command to the NZXT RGB controller
        self.endpoint_out.write(inBuffer)

        if receive:
            # Read the response, an array of byte, .tobytes() gives us a bytearray.
            outBuffer = self.dev.read(self.endpoint_in.bEndpointAddress, 64, 3000).tobytes()

        if self.verbose:
            # Decode and print the zero terminated string response
            print("Send:", inBuffer.hex())
            if receive:
                print("Receive:", outBuffer.hex())

    def set_mode(self, nfan: int, mode: str, color=[0,0,0]):
        # Allocate memory
        inBuffer = bytearray(64)

        # Send Data signal
        inBuffer[0] = 0x2a
        inBuffer[1] = 0x04

        # Device
        inBuffer[2] = FAN_MAP[nfan]
        inBuffer[3] = FAN_MAP[nfan]

        # Mode
        cmode0, cmode1, cmode2, cmode3, cmode4 = COLOR_MODES[mode]
        inBuffer[4] = cmode0
        inBuffer[5] = cmode1
        inBuffer[6] = cmode2

        # Color count
        if mode == "starry-night":
            inBuffer[55] = 0x01
        inBuffer[56] = cmode3
        inBuffer[57] = cmode4
        inBuffer[58] = 0x08
        inBuffer[59] = 0x03

        # Color
        r, g, b = rgb2hex(color[0], color[1], color[2])
        inBuffer[7] = g
        inBuffer[8] = r
        inBuffer[9] = b

        self.send_data(inBuffer)

    def set_direct(self, nfan, index, color):
        Buffer = bytearray(64)
        Buffer[0] = 0x22
        Buffer[1] = 0x10
        Buffer[2] = FAN_MAP[nfan]

        r, g, b = rgb2hex(color[0], color[1], color[2])
        self.colors[index*3] = g
        self.colors[index*3+1] = r
        self.colors[index*3+2] = b
        
        for i in range(8):
            Buffer[4+ i*3]   = self.colors[i*3]
            Buffer[4 + i*3+1] = self.colors[i*3+1]
            Buffer[4 + i*3+2] = self.colors[i*3+2]
            
        self.send_data(Buffer)

        Buffer2 = bytearray(64)
        Buffer2[0] = 0x22
        Buffer2[1] = 0x11
        Buffer2[2] = FAN_MAP[nfan]

        self.send_data(Buffer2)

        Buffer3 = bytearray(64)
        Buffer3[0] = 0x22
        Buffer3[1] = 0xa0
        Buffer3[2] = FAN_MAP[nfan]
        Buffer3[4] = 0x01
        Buffer3[7] = 0x08
        Buffer3[10] = 0x80
        Buffer3[12] = 0x32
        Buffer3[15] = 0x01

        self.send_data(Buffer3)


if __name__ == "__main__":
    import numpy as np
    import time
    nzxt = NZXTUsbDriver(True)

    # Fixed colors
    mode = "fixed"
    color = (0,255, 255)
    nzxt.set_mode(0, mode, color)
    nzxt.set_mode(1, mode, color)
    nzxt.set_mode(2, mode, color)

    # Random colors
    for i in range(1000):
        pos = [0,255]
        nzxt.set_direct(
            np.random.randint(0,3), 
            np.random.randint(0,8), 
            [pos[np.random.randint(0,2)], 
             pos[np.random.randint(0,2)], 
             pos[np.random.randint(0,2)]]
        )
        time.sleep(0.01)
