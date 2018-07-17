import socket
import time

host = '172.20.10.12'
port = 5004

server_addr = (host,port)

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.bind(server_addr)
s.listen(1)

conn,addr = s.accept()
print("Connected to: {}".format(str(addr)))
t1 = time.time()
while True:
    try:
        t = time.time()-t1
        #data = conn.recv(1024)
        #if not data:
        #    print("TCP connection lost. Ending program")
        #    break
        #print("received data: {}".format(data))
        #data = str(data).upper()
        #print(str(t))
        data = " " + str(t)+ " " + str(t)
        conn.send(data)
        time.sleep(0.1)
    except:
        print("\nConnection lost")
        break
print("Closeing program")
conn.close()
s.close()
