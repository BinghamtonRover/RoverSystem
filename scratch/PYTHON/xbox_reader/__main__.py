"""Bring together all implementations"""
from bisect import bisect
from errno import EEXIST
from json import dumps
from math import atan2
from os import mkfifo
from threading import Thread

from . import send_socket
from ..wireless_ap_demo.wap import access_point_manager, stop
from ..button_mappings import button_mappings, get_gamepad

import argparse


instruction_pipe = send_socket.Queue()

STICK_MAX = 32000
STICK_MAX_MAGNITUDE = (2 * (STICK_MAX ** 2)) ** 0.5
TRIGGER_MAX = 16000
STICK_SCALE = 50
TRIGGER_SCALE = 50

# bisect allows numeric table lookups via list indexes
STEP_SCHEMA = ["slow", "medium", "fast"]
STICK_INTERVALS = [STICK_MAX_MAGNITUDE / 3, STICK_MAX_MAGNITUDE * 2 / 3]
TRIGGER_INTERVALS = [TRIGGER_MAX / 3, TRIGGER_MAX * 2 / 3]


def _pipe_instructions(server_queuer, fifo_queuer):
    # asynchronously get the latest instruction from the gamepad's
    # instruction pipe, and push it into both the fifo psuedofile
    # and the server's internal instruction queue
    while True:
        latest_instruction = instruction_pipe.get()
        server_queuer(latest_instruction)
        fifo_queuer(latest_instruction)


def _create_instructions():
    create_container = {key: 0 for key in button_mappings.values()
                        if not (key.endswith('TRIGGER') or key.endswith('STICK'))}
    create_container['LEFT TRIGGER'] = {
        "continuous": 0,
        "step": 0
    }
    create_container['RIGHT TRIGGER'] = {
        "continuous": 0,
        "step": 0
    }
    create_container['LEFT STICK'] = {
        "continuous": 0,
        "step": 0,
        "x": 0,
        "y": 0,
        "angle": 0,
    }
    create_container['RIGHT STICK'] = {
        "continuous": 0,
        "step": 0,
        "x": 0,
        "y": 0,
        "angle": 0
    }

    while True:  # XXX: should we "while gamepad is connected?"
        for event in get_gamepad():
            if event.code == "SYN_REPORT":  # controller is syncing state, let the state be passed
                # x and ys are raw, calculate single vector and step
                for stick in ("LEFT STICK", "RIGHT STICK"):
                    # set the angle of the stick, from pi to negative pi
                    create_container[stick]["angle"] = atan2(
                        create_container[stick]["y"], create_container[stick]["x"])
                    # set the magnitude of the stick
                    create_container[stick]["continuous"] = (
                        (create_container[stick]["y"] ** 2 + create_container[stick]["x"] ** 2) ** 0.5)
                    # set the step and scale the stick
                    create_container[stick]["step"] = STEP_SCHEMA[bisect(
                        STICK_INTERVALS, create_container[stick]["continuous"])]
                    create_container[stick]["continuous"] = (STICK_SCALE * (create_container[stick]["continuous"]) /
                                                             (STICK_MAX_MAGNITUDE))
                # set the step and scale up the values
                for trigger in ("LEFT TRIGGER", "RIGHT TRIGGER"):
                    create_container[trigger]["step"] = STEP_SCHEMA[bisect(
                        TRIGGER_INTERVALS, create_container[trigger]["continuous"])]
                    create_container[trigger]["continuous"] = (TRIGGER_SCALE * (create_container[trigger]["continuous"]) /
                                                               TRIGGER_MAX)
                # instruction setup is complete, queue it to be piped into the fifo and server responses
                instruction_pipe.put(dumps(create_container))
                continue  # skip to next cycle
            action, subaction = event.code.split('_')
            if action == "ABS":
                if subaction in ("X", "Y"):
                    # create the value of the appropriate coordinate
                    create_container["LEFT STICK"][action.lower()] = event.state
                elif subaction in ("RX", "RY"):
                    # [1:] removes the R character
                    create_container["RIGHT STICK"][action.lower()[1:]] = event.state
                elif subaction in ("Z", "RZ"):
                    create_container[{
                        "Z": "LEFT TRIGGER",
                        "RZ": "RIGHT TRIGGER",
                    }[subaction]]["continuous"] = event.state
            else:
                create_container[button_mappings[event.code]] = event.state  # 1 is pressed, 0 is released, -1 is reverse press on hat buttons


def start_server(port):
    # make the server for piping instructions to external clients
    server = send_socket.make_server(port)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    return server, server_thread


def main(network_ssid, network_passwd, port, pipe_file):
    server = server_thread = pipe_instructions = create_instructions = None
    try:
        mkfifo(pipe_file)
    except OSError as e:
        if e.errno != EEXIST:
            # otherwise, fifo already exists, don't worry about this error
            raise
    pipeline = open(pipe_file, 'w')
    try:
        with access_point_manager(network_ssid, network_passwd):
            server, server_thread = start_server(port)
            create_instructions = Thread(target=_create_instructions, daemon=True)
            pipe_instructions = Thread(target=_pipe_instructions,
                                       args=(server.queue_instruction, pipeline.write),
                                       daemon=True)
            pipe_instructions.join()
            create_instructions.join()
            server_thread.join()
    except KeyboardInterrupt:
        if server_thread is not None:
            server.shutdown()
            server_thread.join()
        if pipe_instructions is not None:
            pipe_instructions.join()
        if create_instructions is not None:
            create_instructions.join()
        stop()
    finally:
        pipeline.close()


if __name__ == '__main__':
    # Creates a parser to look at what's passed in to the CL
    parser = argparse.ArgumentParser()
    # Add argument that will be passed in
    # Help is an extra message to guide users and explain variables
    parser.add_argument("ssid", help="Network SSID")
    parser.add_argument("password", help="Network password")
    # Parse the argument as an int so that we can pass directly to main()
    parser.add_argument("port", help="Port to connect to", type=int)
    parser.add_argument("path", help="File path to pipe")
    # Read in arguments
    args = parser.parse_args()
    main(args.ssid, args.password, args.port, args.path)
