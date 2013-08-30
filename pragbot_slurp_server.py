#!/usr/bin/env python
"""Respond to requests on the listening socket by spawning a new client."""

# Copyright (C) 2013 Constantine Lignos
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import os
import signal
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
        if self.client_processes:
            print "Sending shutdown signal to child processes..."
        for process in self.client_processes:
            try:
                os.kill(process.pid, signal.SIGABRT)
            except OSError:
                # Usually this means the process is gone already
                pass

def accept_connections(port, spawner):
    """Spawn a new client each time we receive a valid connection."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
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
