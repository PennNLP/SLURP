"""Communications proxy for reading text input from a socket."""

import sys
import socket
import threading
from threading import Lock
from socket import timeout, error
from Queue import Queue


class CallbackSocket(object):
    """Listen on a socket and call back when a message comes in."""
    name = "commproxy"

    def __init__(self, port, msg_sep="\n", local=False):
        self.msg_sep = msg_sep
        self.callbacks = []
        self.queue = Queue()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('' if not local else 'localhost', port))
        self._sock.listen(5)
        self._callback_lock = threading.Lock()

        # Make a thread that accepts new connections
        accept_thread = threading.Thread(target=self._accept)
        accept_thread.daemon = True
        accept_thread.start()

        # Start the callback thread for synchronously handling requests
        callback_thread = threading.Thread(target=self._pumpmsg)
        callback_thread.daemon = True
        callback_thread.start()

    def _accept(self):
        """Accept connections on the listening port."""
        timed_out = False  # Use this flag to avoid printing when we timeout
        while True:
            if not timed_out:
                print "%s: Waiting for connection..." % self.name
            try:
                conn, addr = self._sock.accept()
                print "%s: Connected to %s" % (self.name, str(addr))
                # TODO: Add support for unique client names
                client = ClientHandler(conn, addr, self)
                client.daemon = True
                client.start()

                timed_out = False
            except timeout:
                timed_out = True
                continue
            except:   # Because any error can occur here during shutdown
                break

    def shutdown(self):
        """Shutdown the socket."""
        print "%s: Shutting down socket." % self.name
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except error:
            pass
        self._sock.close()
        print "%s: Socket closed." % self.name

    def register_callback(self, func):
        """Add a callback to the set of callbacks."""
        with self._callback_lock:
            self.callbacks.append(func)

    def _pumpmsg(self):
        """Read messages and synchronously call back listeners."""
        while True:
            msg, client = self.queue.get()
            # Lock just in case callbacks change while we are pumping
            with self._callback_lock:
                for func in self.callbacks:
                    func(msg, client)


class ClientHandler(threading.Thread):
    """Respond to requests for a particular client."""

    def __init__(self, conn, client_addr, server):
        self._conn = conn
        self._client_addr = client_addr
        self._server = server
        self._send_lock = Lock()
        threading.Thread.__init__(self)

    def run(self):
        """Process client requests."""
        print "%s: Handling requests from %s" % (self.name, str(self._client_addr))
        while True:
            try:
                buff = self._conn.recv(4096)
            except timeout:
                continue
            except error:
                break
            if not buff:
                break
            while buff:
                if self._server.msg_sep:
                    msg, buff = _parse_msg(buff, self._server.msg_sep)
                else:
                    msg, buff = buff, None
                if msg:
                    self._server.queue.put((msg, self))
                else:
                    print "%s: Received an incomplete message: %s" % (self.name, repr(buff))
                    break

        print "%s: Client disconnected." % self.name

    def send(self, msg):
        """Send a message."""
        with self._send_lock:
            self._conn.sendall(msg + self._server.msg_sep)


def _parse_msg(buff, delim):
    """Split a buffer into the msg and the remaining data."""
    idx = buff.find(delim)
    if idx != -1:
        # Throw away the delim when slicing by adding its length
        return (buff[:idx], buff[idx + len(delim):])
    else:
        # Return no message, only buffer
        return (None, buff)


def _test(port):
    """Test the listener."""
    listener = CallbackSocket(port)
    listener.register_callback(_test_callback)

    try:
        _ = raw_input()
    except KeyboardInterrupt:
        pass
    finally:
        print "Exiting..."
        listener.shutdown()


def _test_callback(msg, client):
    """Print the given string."""
    print "callback>", msg
    client.send(msg)

if __name__ == "__main__":
    _test(int(sys.argv[1]))
