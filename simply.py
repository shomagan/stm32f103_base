import sys
import os
import socket
import argparse
import serial
__description__ = 'python simply.py -m 2 -t 5'
def main():
    parser = argparse.ArgumentParser(description=__description__)
    parser.add_argument('-m', '--min_temp', type=int, default=3,
                        help=('minimal temp'
                              '(default: %(default)s)'))
    parser.add_argument('-t', '--triger', type=int, default=5,
                        help=('upper limit trigger when cable enabled'
                              '(default: %(default)s)'))
    args = parser.parse_args()
    try:
        port_name = input("Select COM port: ")  # tutu
        ser = serial.Serial("COM"+port_name)
        ser.baudrate = 115200
        ser.parity = 'O'
        print(ser.name)          # check which port was really used
        sys.stderr.write('--- Miniterm on %s: %d,%s,%s,%s ---\n' % (
            ser.portstr,
            ser.baudrate,
            ser.bytesize,
            ser.parity,
            ser.stopbits,
        ))
    except serial.SerialException as e:
        have_serial = 0
        print("could not open port ", port_name, "\n")
        return
    data = [0xee,args.min_temp,args.triger,0xee]
    ser.timeout = 0.5
    ser.reset_input_buffer()
    ser.write(data)
    for i in range(10):
        recv = ser.read(1)
        if int.from_bytes(recv, byteorder='big') == 0xaa:
            print("parameters succesfully loaded")
            return
        else:
            print(int.from_bytes(recv, byteorder='big'))
    print("recv error try again")
        

if __name__ == "__main__":
    main()
