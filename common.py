# leftover classes, functions with no dependencies shared between multiple modules
from collections import namedtuple
import socket


PIDp = namedtuple('PIDp', ['Kp', 'Ki', 'Kd'])


class ServerSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock: socket.socket = None
        self.conn: socket.socket = None
        self.conn_addr = None

    def __enter__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen()

        print(f'Accepting connections on {self.host}:{self.port}')
        self.conn, self.conn_addr = self.sock.accept()
        self.conn.settimeout(10)  # waiting on blocking calls
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        return self.conn

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.conn:
            self.conn.close()
        if self.sock:
            self.sock.close()
