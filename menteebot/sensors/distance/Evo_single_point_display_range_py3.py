#!/usr/bin/env python3
###### TeraRanger Evo Example Code STD #######
#                                            #
# All rights reserved Terabee France (c) 2018#
#                                            #
############ www.terabee.com #################

import sys

import crcmod.predefined  # To install: pip install crcmod
import serial
import serial.tools.list_ports


def find_evo_port():
    # Find Live Ports, return port name if found, NULL if not
    print("Scanning all live ports on this PC")
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print p # This causes each port's information to be printed out.
        if "5740" in p[2]:
            print("Evo found on port " + p[0])
            return p[0]
    return "NULL"


class Evo:
    def __init__(self):
        self.port = find_evo_port()
        self.serial_port = None
        if self.port == "NULL":
            print("No Evo distance sensor was found")
        else:
            self.open_evo(self.port)

    def open_evo(self, port_name):
        print("Attempting to open port...")
        # Open the Evo and catch any exceptions thrown by the OS
        print(port_name)
        self.serial_port = serial.Serial(port_name, baudrate=115200, timeout=2)
        # Send the command "Binary mode"
        set_bin = (0x00, 0x11, 0x02, 0x4C)
        # Flush in the buffer
        self.serial_port.flushInput()
        # Write the binary command to the Evo
        self.serial_port.write(set_bin)
        # Flush out the buffer
        self.serial_port.flushOutput()
        print("Serial port opened")

    def get_evo_range(self):
        crc8_fn = crcmod.predefined.mkPredefinedCrcFun("crc-8")
        # Read one byte
        data = self.serial_port.read(1)
        if data == b"T":
            # After T read 3 bytes
            frame = data + self.serial_port.read(3)
            if frame[3] == crc8_fn(frame[0:3]):
                # Convert binary frame to decimal in shifting by 8 the frame
                rng = frame[1] << 8
                rng = rng | (frame[2] & 0xFF)
            else:
                print(
                    "CRC mismatch. Check connection or make sure only one program access the sensor port."
                )
                return float("nan")
        # Check special cases (limit values)
        else:
            print("Wating for frame header")
            return float("nan")

        # Checking error codes
        if rng == 65535:  # Sensor measuring above its maximum limit
            dec_out = float("inf")
        elif rng == 1:  # Sensor not able to measure
            dec_out = float("nan")
        elif rng == 0:  # Sensor detecting object below minimum range
            dec_out = -float("inf")
        else:
            # Convert frame in meters
            dec_out = rng / 1000.0
        return dec_out

    def close(self):
        self.serial_port.close()


if __name__ == "__main__":

    print("Starting Evo data streaming")
    # Get the port the evo has been connected to
    evo = Evo()

    if evo.port == "NULL":
        print("Sorry couldn't find the Evo. Exiting.")
        sys.exit()

    while True:
        try:
            print(evo.get_evo_range())
        except serial.serialutil.SerialException:
            print("Device disconnected (or multiple access on port). Exiting...")
            break

    evo.close()
    sys.exit()
