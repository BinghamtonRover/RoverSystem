import argparse
import signal
import socket

from multiprocessing.pool import ThreadPool
from threading import Thread, Condition
from time import sleep


file_data = tuple()
data_lock = Condition()


def update_data(file):
    global file_data
    while True:
        with data_lock:
            with open(file, 'rb') as f:
                data = f.read()
                # our length in binary, as a binary string
                data_length = bin(len(data))[2:].encode("utf-8")
                # pad to be 32 bits / pyBin chars
                data_length = b"0" * (32 - len(data_length)) + data_length
                data = (data_length, data)
            if file_data != data:
                file_data = data
                data_lock.notify_all()
        sleep(5)



def sendfile(client_socket):
    while True:
        with data_lock:
            data_lock.wait()
            # under rare conditions our data can be changed
            # while it's being sent, we send length and data
            # at the same time to avoid a race condition
            # we default to joining an empty tuple in case
            # a threading error causes this thread to wake
            # up before data has been changed
            to_send = b"".join(file_data)
            client_socket.send(b"".join(file_data or tuple()))



def main(port, num_clients, file):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((socket.gethostname(), port))
    sock.listen(num_clients)
    pool = ThreadPool(num_clients)
    t = Thread(target=update_data, args=(file,), daemon=True)
    data_thread_started = False
    # TODO: start our updater in a better way
    # currently we wait for at least one client to connect and then
    # use signal to asynchronously start our updater thread within
    # our main thread. While this works there are definitely better
    # alternatives, however the implementation of these alternatives
    # has to be chosen after our data spec is defined. Better sooner
    # than later, since this lambda is extremely hack-ish and ugly.
    old_alarm_signal = signal.signal(
        signal.SIGALRM,
        lambda signum, frame: (signal.signal(signal.SIGALRM, old_alarm_signal),
                               t.start(), None)[-1])
    try:
        while True:
            client_socket, client_address = sock.accept()
            if not data_thread_started:
                signal.alarm(5)
                data_thread_started = True
            pool.apply_async(sendfile, (client_socket,))
    finally:
        try:
            t.join()
        except:
            # theoretically this thread isn't required to be started,
            # depending on when the user cancels this program (before/after
            pass
        pool.close()
        pool.join()


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Rover Base Station File Sender")
    parser.add_argument("file")
    parser.add_argument("port", type=int)
    parser.add_argument("num_clients", type=int)
    main(**vars(parser.parse_args()))