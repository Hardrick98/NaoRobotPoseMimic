import socket
import torch
import numpy as np
import cv2
import matplotlib.pyplot as plt
import copy
import numpy as np

from src import model
from src import util
from src.body import Body
from src.hand import Hand



def server_program(body, hand):
    # get the hostname
    host = "127.0.0.1"
    port = 5000  # initiate port no above 1024
    
    server_socket = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    server_socket.bind((host, port))  # bind host address and port together
    # configure how many client the server can listen simultaneously
    server_socket.listen(2)
    conn, address = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address))

    with conn:
        image_data = b""
        while True:

            data = conn.recv(4096)
            if not data:
                break
            image_data += data

# Save the received image to disk
            with open("image.jpg", "wb") as file:
                file.write(image_data)

        print("Image received and saved successfully")

    

        oriImg = cv2.imread("image.jpg")
    
        candidate, subset = body(oriImg)
        canvas = copy.deepcopy(oriImg)
        canvas = util.draw_bodypose(canvas, candidate, subset)
        

        """
        # detect hand
        hands_list = util.handDetect(candidate, subset, oriImg)

        all_hand_peaks = []
        for x, y, w, is_left in hands_list:
            # cv2.rectangle(canvas, (x, y), (x+w, y+w), (0, 255, 0), 2, lineType=cv2.LINE_AA)
            # cv2.putText(canvas, 'left' if is_left else 'right', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # if is_left:
                # plt.imshow(oriImg[y:y+w, x:x+w, :][:, :, [2, 1, 0]])
                # plt.show()
            peaks = hand_estimation(oriImg[y:y+w, x:x+w, :])
            peaks[:, 0] = np.where(peaks[:, 0]==0, peaks[:, 0], peaks[:, 0]+x)
            peaks[:, 1] = np.where(peaks[:, 1]==0, peaks[:, 1], peaks[:, 1]+y)
            # else:
            #     peaks = hand_estimation(cv2.flip(oriImg[y:y+w, x:x+w, :], 1))
            #     peaks[:, 0] = np.where(peaks[:, 0]==0, peaks[:, 0], w-peaks[:, 0]-1+x)
            #     peaks[:, 1] = np.where(peaks[:, 1]==0, peaks[:, 1], peaks[:, 1]+y)
            #     print(peaks)
            all_hand_peaks.append(peaks)

        image = util.draw_handpose(canvas, all_hand_peaks)
        """

        cv2.imwrite("estimation.jpg", canvas)

        with open("estimation.jpg", "rb") as file:
            
            image_data = file.read()
    
        conn.sendall(image_data)

        

        conn.close()
        
    server_socket.close()


    print("File sent successfully")


if __name__ == '__main__':
    
    body_estimation = Body('model/body_pose_model.pth')
    hand_estimation = Hand('model/hand_pose_model.pth')


    while True:
        server_program(body_estimation, hand_estimation)