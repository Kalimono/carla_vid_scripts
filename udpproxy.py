import argparse
import socket
import struct

import msgpack

from mirror_main import MirrorMain


def main():
    server, serverport, destination, destinationport = parseCmd()

    trackersocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    trackersocket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, True)
    trackersocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
    trackersocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
    trackersocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)
    trackersocket.connect((server, serverport))
    print("Connected to server at %s:%s" % (server, serverport))

    mirror_main = MirrorMain()

    while True:
        message_id, message_size = readHeader(trackersocket)
        message = read(trackersocket, message_size)

        if message_id == 0xC0FFEE00 and len(message) > 0:

            data = msgpack.unpackb(message, raw=False)
            mirror_main.receive_socket_data(data)


def readHeader(sock):
    header = read(sock, 8)
    message_id = struct.unpack_from("I", header, 0)[0]
    message_size = struct.unpack_from("I", header, 4)[0]
    return message_id, message_size


def read(sock, toread):
    buffer = bytearray(toread)
    view = memoryview(buffer)
    while toread:
        nbytes = sock.recv_into(view, toread)
        if nbytes == 0:
            raise Exception("nothing received")
        view = view[nbytes:]
        toread -= nbytes
    return buffer


def parseCmd():
    parser = argparse.ArgumentParser(
        description="Proxy for getting data from an SDK app and retransmitting it in a different format."
    )
    parser.add_argument(
        "-t",
        dest="server",
        default=["localhost"],
        nargs=1,
        help="address of server (default: localhost)",
    )
    parser.add_argument(
        "-p",
        dest="serverport",
        type=int,
        default=[7000],
        nargs=1,
        help="server port (default: 7000)",
    )
    parser.add_argument(
        "-d",
        dest="destination",
        default=["192.168.0.255"],
        nargs=1,
        help="destination address (default: 192.168.0.255 broadcast)",
    )
    parser.add_argument(
        "-dp",
        dest="destinationport",
        type=int,
        default=[8000],
        nargs=1,
        help="destination port (default: 8000)",
    )
    args = parser.parse_args()
    return (
        args.server[0],
        args.serverport[0],
        args.destination[0],
        args.destinationport[0],
    )


if __name__ == "__main__":
    main()
