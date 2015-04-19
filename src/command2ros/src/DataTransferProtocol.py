__author__ = 'Matt'

# cPicke is faster
try:
    import cPickle as pickle
except ImportError:
    import pickle

# How big of a string should we use to send the data body size
BODY_SIZE_STRING_SIZE = 10


##
# sendData
#
# Description: Send a object over the network
#
# Parameters:
#   socket - the socket to send data over
#   data - the object to send
#
def sendData(socket, data):

    # Convert data into a network friendly object
    data_string = pickle.dumps(data)

    # Create a string to indicate the size of the body
    length = str(len(data_string))
    while len(length) < BODY_SIZE_STRING_SIZE:
        length = '0' + length

    # Send the data string
    socket.send(length)
    socket.send(data_string)

    return


##
# receiveData
#
# Description: Receive a serialized object string and converts it to an object
#
# Parameters:
#   socket - the socket to receive data from
#
# Returns: a data object received over the network
#
def receiveData(socket):

    # first we get a string that says how long the serialized string is
    length = int(socket.recv(BODY_SIZE_STRING_SIZE))

    # If we have received the first part of the data, then we need to get all of it
    # So we will wait for it to all come in
    timeout = socket.gettimeout()
    socket.settimeout(None)

    # We receive and convert a serialized object string to an actual data object
    data_string = socket.recv(length)

    # Return the socket to it's previous blocking state
    socket.settimeout(timeout)

    return pickle.loads(data_string)