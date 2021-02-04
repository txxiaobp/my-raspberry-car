import socket
import time
import threading


class DataSender:
    def __init__(self, gui, ip, port):
        self.gui = gui
        self.ip = ip
        self.port = port
        self.tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client_socket.connect((self.ip, 10999))

    def start(self):
        sendTread = threading.Thread(target=DataSender.dataSendThread, args=(self,))
        sendTread.setDaemon(True)
        sendTread.start()

    def dataSendThread(self):
        while True:
            direction_str = self.gui.getDirectionStr()
            str = ','.join(direction_str)
            str = '*' + str + '&' + '~'
            self.tcp_client_socket.send(bytes(str, encoding="utf-8"))
            print(str)
            self.gui.reset_direction_str()
            time.sleep(0.06)

