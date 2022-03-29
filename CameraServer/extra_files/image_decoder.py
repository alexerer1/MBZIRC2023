#!/usr/bin/env python3

import socket
import time
import cv2
import numpy as np

from contextlib import closing

HOST = '192.168.14.1'  # The server's hostname or IP address
PORT = 4950        # The port used by the server

FRAME_WIDTH = 640 #640
FRAME_HEIGHT = 480 #480
CHANNEL=3

def socketToNumpy(cameraFeed, sockData):
    k=3
    j=cameraFeed.shape[1]
    i=cameraFeed.shape[0]
    sockData = np.fromstring(sockData, np.uint8)
    cameraFeed = np.tile(sockData, 1).reshape((i,j,k))

    return cameraFeed

with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
    print("Ready to connect")
    s.connect((HOST, PORT))
    print("Connected")
    count = 0
    while(1):
        x = input("Command( 1, q): ")
        if( x == '1'):
            print('Sending data')
            for i in range(1,2):
                s.sendall("get_image".encode())
                data = s.recv(921600)
                print('Received = ', repr(data.decode()), '\n')
                time.sleep(1)

        if( x == '2'):
            running = True
            while running:
                i, ptr = (0,0)
                shape = (FRAME_HEIGHT, FRAME_WIDTH, CHANNEL)
                cameraFeed = np.zeros(shape, np.uint8)
                imgSize = cameraFeed.size
                sockData = b''
                result = True

                print("Requesting image")
                start_t = time.time()

                nbytes=s.recv(1000)
                print(nbytes)

                # while imgSize:
                #     nbytes=s.recv(imgSize)
                #     if not nbytes: break; result = False
                #     sockData+=nbytes
                #     imgSize-=len(nbytes)

                print("Took ", (time.time()-start_t), " sec to get and image" )

                if result:
                    cameraFeed = socketToNumpy(cameraFeed, sockData)

                    # Create a window for display.
                    cv2.namedWindow("server");
                    cv2.imshow("server", cameraFeed)
                    key = cv2.waitKey(30)
                    running = key

                    # esc
                    if key==27:
                        running =False
                else : running =False
        elif(x == 'q'):
            break

s.close()