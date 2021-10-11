from py_streamer.hololens2_simpleclient import send_file_to_HL
import argparse
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test tcp')
    parser.add_argument("--file_path", required=True, default=False,
                        help="file path to send (txt file) ")
    args = parser.parse_args()
    file_path = args.file_path
    send_file_to_HL(file_path)