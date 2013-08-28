#!/usr/bin/env python
"""Respond to requests on the listening socket by spawning a new client."""

import sys
from multiprocessing import Process
import socket
from socket import timeout, error

import pragbot_client

STARTUP_MESSAGE = "PRAGBOT"
RESPONSE = "OK"


class Spawner:
    """Manage the spawning of pragbot clients."""

    def __init__(self):
        self.client_processes = []

    def spawn_client(self):
        """Create a new client."""
        print "Spawning client..."
        new_client = Process(target=pragbot_client.main)
        self.client_processes.append(new_client)
        new_client.start()

    def shutdown(self):
        """Shut down all child processes."""
        for process in self.client_processes:
            process.terminate()


def accept_connections(port, spawner):
    """Spawn a new client each time we receive a valid connection."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('localhost', port))
    sock.listen(5)

    timed_out = False  # Use this flag to avoid printing when we timeout
    while True:
        if not timed_out:
            print "Waiting for connection..."
        try:
            conn, addr = sock.accept()
            print "Connected to %s" % str(addr)
            timed_out = False

            # Read off the message
            try:
                buff = conn.recv(4096)
            except timeout:
                print "Timed out when receiving data"
            except error:
                print "Error receiving data"
            else:
                if buff == (STARTUP_MESSAGE + "\n"):
                    spawner.spawn_client()
                    conn.sendall(RESPONSE + "\n")
                else:
                    print "Received unexpected message:", repr(buff)
        except timeout:
            timed_out = True
            continue
        except error:
            # The connection is completely broken, exit
            print "Error making connection"
            break
        else:
            # Close up this connection
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except error:
                pass
            conn.close()

    # Close the listening socket
    sock.close()


def main():
    """Listen on a socket and open a new connection."""
    try:
        port = int(sys.argv[1])
    except (IndexError, ValueError):
        print "Usage: pragbot_slurp_server port"
        sys.exit(64)

    # Spawn clients
    spawner = Spawner()
    try:
        accept_connections(port, spawner)
    except KeyboardInterrupt:
        pass
    finally:
        spawner.shutdown()


if __name__ == "__main__":
    main()
