========================
=== Xbox Remote Demo ===
========================

The purpose of this demo is to send Xbox controller state from one computer to another via an ethernet connection.
The demo is broken into four parts, as follows.

The demo uses the xbox-remote-demo branch. The code will go in scratch/PYTHON/xbox_remote_demo.

==[ 1. Reading Controller State ]==

One computer in the demo will be connected to an Xbox controller. This can be done with a wire or with bluetooth, as
long as the controller is connected to the OS.

This part consists of a Python file which exposes a single function. This function will take one parameter, which is
a callback that takes a single parameter of type ControllerState (found in controller_state.py). This single
function should loop infinitely, keeping a record of the current Xbox state (in a ControllerState object). It should
listen for updates from the controller, update its record, then pass said record to the callback. The python module
'inputs' should be used for this (installed with 'pip install inputs' or 'pip3 install inputs'). Note that the inputs
module does not normalize controller values; this will have to be done within this function. See
https://github.com/ZeldaZach/BinghamtonRover2017/blob/HaxagonusD-patch-1/Button%20Mappings.py for how to read and
interpret the controller button presses.

==[ 2. Sending Controller State ]==

This part will, on a clock, continuously send UDP packets from the computer with the Xbox controller to the one without.
Each packet will contain the information in ControllerState, in the order in which the buttons are defined in
controller_state.py. The state sent across the wire will be updated by part 1 above when the callback is called.

Packet format:
    - For each button in ControllerState:
        - A single unsigned byte denoting the button (from 0 to 18 in the order the buttons are listed in
          controller_state.py).
        - Four bytes denoting the button state. For buttons with only on and off states, this will be an unsigned
          integer (either 0 or 1). For buttons with continuous states, this will be a signed 32-bit IEEE floating point
          number between -1 and 1.

Packet length (in bytes): 19 * 5 = 95 bytes.

==[ 3. Receiving Controller State ]==

This part will run on the computer without the Xbox controller. It will listen for UDP packets at a specific port.
When it receives one, it must deserialize it into a ControllerState object, and pass said object to part 4. Expect
the packet format outlined in part 2.

==[ 4. Displaying Controller State ]==

This part will run on the computer without the Xbox controller. It will expose a single function that takes a
ControllerState parameter. Each time this is called, a visual display which represents the controller's state should
be updated. This display should be visual.

===================
=== Other Notes ===
===================

Make sure you don't commit your .idea/ folder! As long as you never add it to Git, you will be fine.
Also, make sure you are on the xbox_remote_demo branch, and not on master!

Make sure you make a lot of comments! There should be a docstring for each file and function! There should be a comment
for every operation you do. This doesn't have to be for every line... if three lines do the same basic thing you may
have a single comment for all of them. Basically, Zach should be able to understand everything that is going on without
having to understand how the libraries we are using work. We don't want him to chew us out!

Also, use the naming conventions that Zach outlined in the presentation (its in the Drive folder). This way, variables
are self-documenting. This does not mean comments aren't necessary... the variable names say what, the comments say why.
Here's an example of the naming conventions in use:

-------------------------------------------------------------------------------
def this_is_a_function(an_num_people, aas_people_names, ao_controller_state):
    ls_num_people_display_string = "There are {} people.".format(an_num_people)
    # ...
-------------------------------------------------------------------------------

The conventions are as follows:

    1. Start with a prefix denoting the scope of the variable. For local variables (within functions), use 'l'. For
       function arguments (parameters), use 'a'. For class members, use 'c'. For global variables and static class
       variables, use 'g'.
    2. If the variable is an array, add an 'a'.
    3. Denote the type. 'f' for floats, 'n' for ints, 'b' for booleans, 's' for strings, 'o' for objects.

The conventions don't work by default for dictionaries, since one can only define a single type. If you have a
dictionary, use 'd' for the type. That should be enough.