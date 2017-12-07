from json import dumps
from queue import Queue
from socketserver import ThreadingTCPServer, BaseRequestHandler


class delegate_request(object):
    class RequestError(Exception):
        def __init__(self, code, reason):
            self.code = code
            self.reason = reason

    def __get__(self, handler, handlertype):
        # attach the handler to this descriptor
        self.handler = handler
        return self

    def __call__(self, request_op):
        # get the function to run by opcode and execute
        # if the opcode doesn't exist or an error propogates,
        # give the client this error
        try:
            return getattr(self, request_op)()
        except self.RequestError as e:
            return self.FAIL(e)
        except AttributeError: # OPCODE not handled
            return self.FAIL(self.RequestError(400, "Invalid opcode"))

    def GET_INSTR(self):
        # fetch the latest instruction pushed into
        # the queue  and send it to the client
        instruction = self.handler.server.instruction_queue.get()
        self.handler.request.sendall(dumps(instruction))

    def CLOSE_SRVR(self):
        self.handler.server.shutdown()

    def FAIL(self, exc_obj):
        # give the client this error
        self.handler.request.sendall(dumps([
            exc_obj.code, exc_obj.reason
        ]))


class RoverRequestHandler(BaseRequestHandler):
    def handle(self):
        # pass on the request to the delegator. Reads
        # a maximum of 1024 bytes. We can technically
        # minimize this but 1024 is a standard number
        # of bytes to read
        self.delegator(self.request.recv(1024).strip())

    delegator = delegate_request()


class RoverInstrtuctionServer(ThreadingTCPServer):
    request_queue_size = 1  # only one client, the rover
    instruction_queue = Queue()  # FIFO queue for instruction pipeline

    def queue_instruction(self, instruction):
        # convenience function to add an item to our pipeline
        self.instruction_queue.put(instruction)


def make_server(port):
    return RoverInstrtuctionServer(("", port), RoverRequestHandler)