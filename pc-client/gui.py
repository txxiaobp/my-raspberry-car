import tkinter as tk
import threading
import time


left = 0
right = 1
up = 2
down = 3
mouseX = 4
mouseY = 5
halt = 6


class GUI:
    def __init__(self, dataSender):
        self.dataSender = dataSender
        self.directionLock = threading.Lock()
        self.directionStr = ['0', '0', '0', '0', '0', '0', '0']

    def getDirectionStr(self):
        return self.directionStr

    def start(self):
        window = tk.Tk()
        frame = tk.Frame(window, width=1500, height=1500)
        frame.bind("<Up>", self.upCall)  # 触发的函数
        frame.bind("<Down>", self.downCall)  # 触发的函数
        frame.bind("<Left>", self.leftCall)  # 触发的函数
        frame.bind("<Right>", self.rightCall)  # 触发的函数
        frame.bind("<Motion>", self.mouseMove)
        frame.bind("<Key>", self.key)
        frame.focus_set()  # 必须获取焦点
        frame.pack()
        window.mainloop()

    def reset_directionStr(self):

        self.directionLock.acquire()
        self.directionStr[left] = '0'
        self.directionStr[right] = '0'
        self.directionStr[up] = '0'
        self.directionStr[down] = '0'
        self.directionStr[halt] = '0'
        self.directionLock.release()

    def upCall(self, value):
        self.directionLock.acquire()
        self.directionStr[up] = '1'
        self.directionLock.release()

    def downCall(self, value):
        self.directionLock.acquire()
        self.directionStr[down] = '1'
        self.directionLock.release()

    def leftCall(self, value):
        self.directionLock.acquire()
        self.directionStr[left] = '1'
        self.directionLock.release()

    def rightCall(self, value):
        self.directionLock.acquire()
        self.directionStr[right] = '1'
        self.directionLock.release()

    def haltCall(self, value):
        self.directionLock.acquire()
        self.directionStr[halt] = '1'
        self.directionLock.release()

    def mouseMove(self, event):

        if self.last_x is None:
            self.last_x = event.x_root
            self.last_y = event.y_root
            self.directionStr[mouseX] = '0'
            self.directionStr[mouseY] = '0'
        else:
            self.directionStr[mouseX] = str(event.x_root - self.last_x)
            self.directionStr[mouseY] = str(event.y_root - self.last_y)
            self.last_x = event.x_root
            self.last_y = event.y_root

    def key(self, event):
        if event.char == 'w':
            self.upCall(0)
        elif event.char == 's':
            self.downCall(0)
        elif event.char == 'a':
            self.leftCall(0)
        elif event.char == 'd':
            self.rightCall(0)
        elif event.char == 'q':
            self.haltCall(0)
        time.sleep(0.01)