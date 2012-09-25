"""Communications proxy for reading text input from a socket."""

import sys
import socket
import threading
from socket import timeout, error
from Queue import Queue


class CallbackSocket(object):
    """Listen on a socket and call back when a message comes in."""
    name = "commproxy"

    def __init__(self, port, msg_sep="\n", local=False):
        self.msg_sep = msg_sep
        self.callbacks = []
        self._queue = Queue()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('' if not local else 'localhost', port))
        self._sock.listen(1)
        self._conn = None
        self.client_addr = None
        self._callback_lock = threading.Lock()
        self._send_lock = threading.Lock()

        # Make a thread that keeps the connection alive
        accept_thread = threading.Thread(target=self._accept)
        accept_thread.daemon = True
        accept_thread.start()

        # Start the callback thread
        callback_thread = threading.Thread(target=self._pumpmsg)
        callback_thread.daemon = True
        callback_thread.start()

    def _accept(self):
        """Accept connections on the listening port."""
        waiting = False # Use this flag to avoid printing when we timeout
        while True:
            if not waiting:
                print "%s: Waiting for connection..." % self.name
                waiting = True
            try:
                self._conn, addr = self._sock.accept()
                self.client_addr = addr
                print "%s: Connected to %s" % (self.name, str(addr)) 
            except timeout:
                self._conn = None
                self.client_addr = None
                continue
            except: # Because any error can occur here during shutdown
                break
                
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
                    if self.msg_sep:
                        msg, buff = _parse_msg(buff, self.msg_sep)
                    else:
                        msg, buff = buff, None
                    if msg:
                        self._queue.put(msg)
                    else:
                        print "%s: Received an incomplete message: %s" % (self.name, repr(buff)) 
                        break
                
            print "%s: Client disconnected." % self.name
            self._conn = None
            self.client_addr = None
            waiting = False

    def shutdown(self):
        """Shutdown the socket."""
        print "%s: Shutting down socket." % self.name
        self._sock.shutdown(socket.SHUT_RDWR)
        self._sock.close()
        print "%s: Socket closed." % self.name

    def send(self, msg):
        """Send a message."""
        if self._conn:
            with self._send_lock:
                # Pylint reports a false positive here
                # pylint: disable=E1101
                self._conn.sendall(msg + self.msg_sep)

    def register_callback(self, func):
        """Add a callback to the set of callbacks."""
        with self._callback_lock:
            self.callbacks.append(func)
            
    def is_connected(self):
        """Report whether we have a client connected."""
        return bool(self._conn)

    def _pumpmsg(self):
        """Read messages and synchronously call back listeners."""
        while True:
            msg = self._queue.get()
            with self._callback_lock:
                for func in self.callbacks:
                    func(msg)


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


def _test_callback(msg):
    """Print the given string."""
    print "callback>", msg

if __name__ == "__main__":
    _test(int(sys.argv[1]))
