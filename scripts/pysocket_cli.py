#!/usr/bin/env python
import socket
import os, os.path
import time
import random
 
print "Connecting..."
if os.path.exists( "/tmp/soundloc_sockets" ):
    client = socket.socket( socket.AF_UNIX, socket.SOCK_STREAM )
    client.connect( "/tmp/soundloc_sockets" )

    # Send data
    while True:
        ts = time.time()
        angle = random.randint(-90,90)
            
        message = str(ts)+"|"+str(angle)
        print(message)
        client.sendall(message)
        #time.sleep(0.0009)
        time.sleep(1)
else:
  print "Couldn't Connect!"
print "Done"
