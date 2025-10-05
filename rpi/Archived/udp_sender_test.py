# # ping esp 
# import socket, json, time
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# esp_ip = "192.168.1.80"
# port = 5005

# msg = {"seq":1, "cmd":{"v":0.2, "w":0.1}, "state":"FOLLOW"}
# sock.sendto(json.dumps(msg).encode(), (esp_ip, port))
# print("Sent test packet")
