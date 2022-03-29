#!/usr/bin/env python3

import socket
import time

from contextlib import closing

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 7740        # The port used by the server

with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
    print("Ready to connect")
    s.connect((HOST, PORT))
    print("Connected")
    count = 0
    while(1):
        x = input("Command( 1, 2, 3, 4, q): ")
        if( x == '1'):
            for i in range(1,10):
                s.sendall("get_pose".encode())
                data = s.recv(1024)
                print('Received = ', repr(data), '\n')
                time.sleep(0.01)
        elif( x == '2'):
            for i in range(1,10):
                s.sendall("what!?".encode())
                time.sleep(0.1)
        elif( x == '3'):
            for i in range(1,10):
                s.sendall("set_atti: 1.2 2.3 5.1 2.1 6.3 end".encode())
                time.sleep(0.1)
        elif( x == '4'):
            for i in range(1,10):
                s.sendall("set_loc_type: 1 end".encode())
                data = s.recv(1024)
                print('Received = ', repr(data), '\n')
                time.sleep(0.1)
        elif(x == 'q'):
            break

s.close()