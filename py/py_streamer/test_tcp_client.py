import argparse
import asyncio
import socket
import os
# sending file (suppose to be txt file) to HL, old
def send_file_to_HL_0(file_path_to_send, host, loading_port):
    if file_path_to_send is None:
        print('There is no file path for sending back to HL!!')
        return
    packet_size = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, loading_port))
    #s.connect((LOCAL_HOST, LOADING_PORT))
    filetosend = open(file_path_to_send, "rb")
    data = filetosend.read(packet_size)
    while data:
        print("Sending after reg stream...")
        s.send(data)
        data = filetosend.read(packet_size)
    filetosend.close()
    print("Done Sending after reg stream.")

    s.shutdown(2)
    s.close()
    print("close connection")
    # Done :)

# sending file (suppose to be txt file) to HL
def send_file_to_HL(file_path_to_send, host, loading_port):
    if file_path_to_send is None:
        print('There is no file path for sending back to HL!!')
        return
    loop = asyncio.get_event_loop()
    loop.run_until_complete(tcp_echo_client(file_path_to_send, loop, host, loading_port))
    loop.close()

    print("close connection")
    # Done :)

async def tcp_echo_client(file_path_to_send, loop, host, loading_port):
    #The loop argument is deprecated since Python 3.8
    #reader, writer = await asyncio.open_connection(host, loading_port, loop=loop)
    reader, writer = await asyncio.open_connection(host, loading_port)

    filetosend = open(file_path_to_send, "r", encoding="utf-8")
    # get the size of file
    size_bytes_of_txt_file = os.path.getsize(file_path_to_send)
    print('Size of file is', size_bytes_of_txt_file, 'bytes')
    content = filetosend.read()
    # d = '\n'
    # lines = [e + d for e in content.split(d) if e]
    # for line in lines:
    #     print("Sending after reg stream...")
    #     writer.write( str(len(line)).encode() )
    #     writer.write(line.encode())
    print("Sending after reg stream...")
    writer.write(str(size_bytes_of_txt_file).encode())
    writer.write(content.encode())
    # data = filetosend.read(packet_size) #sending in packet size
    # while data:
    #     print("Sending after reg stream...")
    #     writer.write(data.encode())
    #     data = filetosend.read(packet_size)
    filetosend.close()
    print("Done Sending after reg stream.")

    #data_rec = await reader.read(100)
    #print('Received: %r' % data_rec.decode())

    print('Close the socket')
    writer.close()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='test tcp')
    parser.add_argument("--file_path", required=True, default=False,
                        help="file path to send (txt file) ")
    args = parser.parse_args()
    file_path = args.file_path
    LOADING_PORT = 13000
    HOST = '10.0.0.4'
    LOCAL_HOST = '127.0.0.1'
    #send_file_to_HL_0(file_path, HOST, LOADING_PORT) # old try

    # THIS CODE WORKS SENDING STRING MESSAGE TO HOLOLENS

    send_file_to_HL(file_path, LOCAL_HOST, LOADING_PORT)
