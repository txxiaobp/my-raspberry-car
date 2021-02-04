import threading
import socket
import queue


class DataReceiver:
    def __init__(self, motor, servo):
        self.motor = motor
        self.servo = servo
        self.dataQueue = queue.Queue()

    def start(self):
        left = 0
        right = 1
        up = 2
        down = 3
        mouseX = 4
        mouseY = 5
        halt = 6

        ip = self.getHostIp()
        tcpServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcpServerSocket.bind((ip, 10999))
        tcpServerSocket.listen(120)
        new_client_socket, client_addr = tcpServerSocket.accept()

        recvThread = threading.Thread(target=DataReceiver.receiveDataThread, args=(self,new_client_socket,))
        recvThread.setDaemon(True)
        recvThread.start()

        while True:
            directionStr = self.dataQueue.get().split(',')
            if directionStr[left] == '1':
                self.motor.goLeft()
            if directionStr[right] == '1':
                self.motor.goRight()
            if directionStr[up] == '1':
                self.motor.goUp()
            if directionStr[down] == '1':
                self.motor.goDown()
            if directionStr[halt] == '1':
                self.motor.stop()

            servoMoveX = int(directionStr[mouseX])
            servoMoveY = int(directionStr[mouseY])
            self.servo.servoMove(servoMoveX, servoMoveY)

        tcpServerSocket.close()

    def receiveDataThread(self, socket):

        startStr = '*'
        endStr = '&'
        splitStr = '~'
        uncompleteStr = str()

        while True:
            recvData = socket.recv(64).decode("utf-8")
            dataArray = recvData.split(splitStr)

            for data in dataArray:
                if len(data) < 1:
                    continue
                if data[0] != startStr:
                    tmpStr = uncompleteStr + data
                    self.dataQueue.put(tmpStr[1:-1])
                    uncompleteStr = str()
                elif data[-1] != endStr:
                    uncompleteStr = data
                else:
                    self.dataQueue.put(data[1:-1])

    def getHostIp(self):
        """
        获取本机ip地址
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
        finally:
            s.close()
        return ip
