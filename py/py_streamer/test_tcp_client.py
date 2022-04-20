import argparse
import asyncio
import socket
import os
from py_streamer.zmq_server import *


# sending file (suppose to be txt file) to HL, old
def send_file_to_HL_0(file_path_to_send, host, loading_port):
    if file_path_to_send is None:
        print('There is no file path for sending back to HL!!')
        return
    loop = asyncio.get_event_loop()
    loop.run_until_complete(tcp_echo_client(file_path_to_send, loop, host, loading_port))
    loop.close()

    print("close connection")
    # Done :)
async def tcp_echo_client(file_path_to_send, loop, host, loading_port):
    # The loop argument is deprecated since Python 3.8
    # reader, writer = await asyncio.open_connection(host, loading_port, loop=loop)
    reader, writer = await asyncio.open_connection(host, loading_port)

    filetosend = open(file_path_to_send, "r", encoding="utf-8")
    # get the size of file
    size_bytes_of_txt_file = os.path.getsize(file_path_to_send)

    content = filetosend.read()
    # content ="# Created by Open3D\nv 0 0 0\nv 0 1 0\nv 1 1 0\nv 1 0 0\nv 1 1 1\nv 0 1 1\nv 0 0 1\nf 1 2 3 4\nf 1 2 5 6\n"
    print('Size of file is', size_bytes_of_txt_file, 'bytes')
    # print(content)

    print("Sending after reg stream...")

    writer.write(str(size_bytes_of_txt_file).encode())
    # await writer.drain()

    writer.write(content.encode())
    # await writer.drain()
    writer.write_eof()
    await writer.drain()
    # data = filetosend.read(packet_size) #sending in packet size
    # while data:
    #     print("Sending after reg stream...")
    #     writer.write(data.encode())
    #     data = filetosend.read(packet_size)
    filetosend.close()
    print("Done Sending after reg stream.")

    # data_rec = await reader.read(100)
    # print('Received: %r' % data_rec.decode())

    print('Close the socket')
    writer.close()


# sending file (suppose to be txt file) to HL
def send_file_to_HL(file_path_to_send, host, loading_port):
    if file_path_to_send is None:
        print('There is no file path for sending back to HL!!')
        return
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    zmq_server = ServerZMQ(port=loading_port)
    zmq_server.init_server()
    zmq_server.test_send_PUBSUB(content)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test tcp')
    parser.add_argument("--file_path", required=True, default=False,
                        help="file path to send (txt file) ")
    args = parser.parse_args()
    file_path = args.file_path
    LOADING_PORT = 12345
    HOST = '10.0.0.1'
    LOCAL_HOST = '127.0.0.1'
    # send_file_to_HL(file_path, HOST, LOADING_PORT)
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    test_zmq_server(content, port=LOADING_PORT)
