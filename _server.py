import os
import socket
import time


# this is a subscriber (server) script, getting a pose from a client (from another pc in network) and it moves the panda according to the received poses

x = 0.4 # standard goal values (float)
y = 0.0
z = 0.2


host = '132.72.103.111'        # Symbolic name meaning all available interfaces
port = 12346     # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host, port))

print host , port
s.listen(1)

while True:
    conn, addr = s.accept()
    print('Connected by', addr)
    while True:
        try:
            data = conn.recv(1024)

            if not data: break

            print "Client sent the pose: " + data
            #raw_input("Enter to execute")
            cmd = "/home/franka1/franka/libfranka/build/examples/generate_cartesian_pose_motion 172.16.1.2 " + data
            #print(cmd)
            print " "
            print "Calling cartesian pose motion script"
            os.system(cmd) # run the cmd in bash
            time.sleep(1)

            f = open("/home/franka1/franka/libfranka/data/dataIO.txt", "r")
            report = f.read()
             
            conn.send("Server Says: Panda finished moving")
            conn.send(report)

            print("Waiting for new pose... ... ... ... ... ... ... ... ... ... ...")

        except socket.error:
            print "Error Occured."
            break

    conn.close()
