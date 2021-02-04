from gui import GUI
from senddata import DataSender


def main():

    ip = "192.168.43.89"
    port = 10999

    gui = GUI()
    dataSender = DataSender(gui, ip, port)

    dataSender.start()
    gui.start()

if __name__ == '__main__':
    main()




