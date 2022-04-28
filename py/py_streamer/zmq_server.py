import random
import time
import zmq

PORT_ZMQ = 12345


class ServerZMQ():
    def __init__(self, port=PORT_ZMQ):
        self.socket = None
        self.port = port

    def init_server(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.port}")

    def listen(self):  # on REQ\RES
        #  wait request from client
        message_rx = self.socket.recv()
        print(f"Received request: {message_rx}")

    def send_PUBSUB(self, str_message="default message"):
        self.socket.send_string(str_message)
        print(f'Done sending.')

    def test_send_PUBSUB(self, str_message="default message"):
        i = 0
        while True:
            # send message every time_interval seconds
            message = str_message
            print(f'Sending number {i}')
            #print(message)
            print()
            self.socket.send_string(message)
            time.sleep(15)
            i+=1

    def test_send_REQRES(self):  # on REQ\RES
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


def test_zmq_server(str_message, port=PORT_ZMQ):
    zmq_server = ServerZMQ(port=port)
    zmq_server.init_server()
    zmq_server.test_send_PUBSUB(str_message)


if __name__ == '__main__':
    # ZMQ server
    test_zmq_server(str_message="default main check", port=PORT_ZMQ)
