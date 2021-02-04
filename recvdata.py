def get_host_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()

    return ip


def recv_data_thread(socket):
    start_str = '*'
    end_str = '&'
    split_str = '~'
    uncomplete_str = str()
    while True:
        recv_data = socket.recv(64).decode("utf-8")
        data_array = recv_data.split(split_str)
        length = len(data_array)

        for data in data_array:
            if len(data) < 1:
                continue
            if data[0] != start_str:
                tmp_str = uncomplete_str + data
                data_queue.put(tmp_str[1:-1])
                uncomplete_str = str()
            elif data[-1] != end_str:
                uncomplete_str = data
            else:
                data_queue.put(data[1:-1])


def recv_thread():
    left = 0
    right = 1
    up = 2
    down = 3
    mouseX = 4
    mouseY = 5
    halt = 6

    ip = get_host_ip()
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server_socket.bind((ip, 10999))
    tcp_server_socket.listen(120)
    new_client_socket, client_addr = tcp_server_socket.accept()
    threading.Thread(target=recv_data_thread, args=(new_client_socket,)).start()

    while True:
        direction_str = data_queue.get().split(',')
        if direction_str[left] == '1':
            leftCallBack()
        if direction_str[right] == '1':
            rightCallBack()
        if direction_str[up] == '1':
            upCallBack()
        if direction_str[down] == '1':
            downCallBack()
        if direction_str[halt] == '1':
            haltCallBack()

        camera_move_x = int(direction_str[mouseX])
        camera_move_y = int(direction_str[mouseY])
        camera_move(camera_move_x, camera_move_y)

    server_socket.close()