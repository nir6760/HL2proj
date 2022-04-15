import random
import time
import zmq
PORT_ZMQ = 12345


class ServerZMQ():
    def __init__(self):
        self.socket = None

    def init_server(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{PORT_ZMQ}")

    def listen(self):
        #  wait request from client
        message_rx = self.socket.recv()
        print(f"Received request: {message_rx}")

    def test_send(self):
        while True:
            #  wait request from client
            message_rx = self.socket.recv()
            print(f"Received request: {message_rx}")

            #  do something
            time.sleep(1)

            #  reply to client
            message = str(random.uniform(-1.0, 1.0)) + " " + str(random.uniform(-1.0, 1.0))
            print(message)
            self.socket.send_string(message)

if __name__ == '__main__':
    # ZMQ server
    zmq_server = ServerZMQ()
    zmq_server.init_server()
    zmq_server.listen()



