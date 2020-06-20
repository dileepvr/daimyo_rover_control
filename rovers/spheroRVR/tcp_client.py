# Python program to implement client side of chat room.
import socket
import select
import sys

IP_address = 'localhost'
Port = 8082

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
if len(sys.argv) not in [1, 3]:
    print("Correct usage: script, IP address, port number")
    exit()
if len(sys.argv) == 3:
    IP_address = str(sys.argv[1])
    Port = int(sys.argv[2])

server.connect((IP_address, Port))
running = True

while running:
    soclist = [sys.stdin, server]
    inlist, outlist, exlist = select.select(soclist, [], [])
    for inready in inlist:
        if inready == server:
            message = server.recv(2084)
            if message:
                print(message.decode())
            else:
                sys.stdout.write("<server died.>\n")
                server.close()
                running = False
        else:
            message = sys.stdin.readline().split()[0]
            if message == 'quit':
                server.send('<closing>'.encode())
                server.close()
                running = False
            else:
                server.send(message.split()[0].encode())
                sys.stdout.write("<sent.>\n")
                sys.stdout.flush()
try:
    server.close()
except:
    pass
