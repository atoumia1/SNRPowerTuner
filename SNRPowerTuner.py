import sys
import time
import argparse
import serial
import RNS

# ------------ Global variables ------------
APP_NAME = "SNRPowerTuner"  # Application name
SERIAL_PORT = "/dev/ttyACM0"  # Serial port for the radio
SERVER_LINK = None  # Stores the link to the server
CLIENT_LINK = None  # Stores the link to the client
SNR_VALUE = None  # SNR value received from the server
READY_MESSAGE = False  # Flag to indicate where it is redy to receive messages
ALLOW_DECREMENT = True  # Flag to indicate if the client is allowed to decrement the power
STABLE_COUNTER = 0  # The number of times the TX power has stabilised
MAX_TX_POWER = 22  # The maximum TX power
MIN_TX_POWER = 1  # The minimum TX power
SNR_THRESHOLD = -1  # The SNR threshold
# ------------------------------------------

# Bytes to be used as the first byte of packets to indicate the type of packet
class BYTE:
    EMPTY = 0xA0  # Empty packet
    SNR = 0xA1  # SNR packet
    TX = 0xA2  # TX packet
    MSG = 0xA3  # Message packet

# Defines constants used in the KISS protocol for framing and commands
class KISS:
    FEND = 0xC0  # Used to delimit the start and end of a command
    CMD_TXPOWER = 0x03  # Command for setting transmission power

##########################################################
#### Shared Code #########################################
##########################################################

# Opens the serial port
def OpenSerialPort(baudrate = 115200):
    try:
        return serial.Serial(SERIAL_PORT, baudrate, timeout = 1)  # Opens the serial port
    except serial.SerialException as e:
        RNS.log(f"Failed to open serial port: {SERIAL_PORT}\n   {e}")
        sys.exit(1)

# Sets the transmission power of the radio
def SetTXPower(serialPort, txPower):
    txPowerCommand = bytes([KISS.FEND, KISS.CMD_TXPOWER, txPower, KISS.FEND])  # Command to set the transmission power
    serialPort.write(txPowerCommand)  # Sends the command to the radio over the serial port
    RNS.log(f"Set TX power to {txPower} dBm")

# Creates a command packet
def CreateCommand(byte, data=''):
    if data:
        dataBytes = str(data).encode("utf-8")  # Converts the data to bytes
    else:
        dataBytes = b''  # Empty data

    return bytes([byte]) + dataBytes  # Returns the command packet

# Runs when a packet is received by the server
def PacketReceived(message, packet):
    global SNR_VALUE, READY_MESSAGE
    commandByte = message[0]  # The first byte of the message
    data = message[1:]  # The rest of the message

    if commandByte == BYTE.EMPTY:
        ProcessSNR(packet)  # Will calculate the SNR value of the packet and send it to the client
    elif commandByte == BYTE.SNR:
        try:
            SNR_VALUE = float(data.decode("utf-8"))  # The SNR value received from the client
            RNS.log(f"Received SNR value: {SNR_VALUE}dB\n")
        except ValueError as e:
            RNS.log(f"Error processing SNR value: {e}")
    elif commandByte == BYTE.TX:
        try:
            optimalTX = int(data.decode("utf-8"))  # The optimal TX power received from the client
            serialPort = OpenSerialPort()  # Opens the serial port
            SetTXPower(serialPort, optimalTX)  # Sets the transmission power
            serialPort.close()  # Closes the serial port
            RNS.log(f"Set TX power to optimal value: {optimalTX} dBm\nPress enter to start messaging")
            READY_MESSAGE = True  # Sets the flag to indicate that messaging is ready
        except ValueError as e:
            RNS.log(f"Error setting TX power: {e}")
    elif commandByte == BYTE.MSG:
        ReceivedMessage(data)  # Will process the message received from the client
    else:
        RNS.log(f"Unknown command byte: {commandByte}")

# Function called when ready to send messages
def StartMessaging(link):
    if link is None:
        RNS.log("No link to send messages on")  # If there is no link, return
        return

    RNS.log("Messaging mode active")
    try:
        while True:
            message = input("> ")  # Gets the message from the user
            if message != "":
                messageData = CreateCommand(BYTE.MSG, message)  # Creates a message packet
                RNS.Packet(link, messageData).send()  # Sends the message packet
    except TypeError as e:
        RNS.log(f"Error in messaging mode: {e}")
    finally:
        RNS.log("Exiting messaging mode")

# Function called when message packet recieved
def ReceivedMessage(data):
    if READY_MESSAGE:
        message = data.decode("utf-8")  # The message received from the client
        RNS.log(f"Message received: {message}")
        sys.stdout.flush()  # Flushes the output
    else:
        RNS.log("Message received but messaging not ready")

##########################################################
#### Server Code #########################################
##########################################################

# Main server function
def Server(configpath):
    RNS.Reticulum(configpath)  # Initialises Reticulum
    serverIdentity = RNS.Identity()  # Creates a new identity
    serverDestination = RNS.Destination(
        serverIdentity,
        RNS.Destination.IN,
        RNS.Destination.SINGLE,
        APP_NAME,
        "PowerTuner",
    )
    serverDestination.set_link_established_callback(ClientConnected)
    ServerLoop(serverDestination)

# Main server loop
def ServerLoop(destination):
    RNS.log(f"Power Tuner server started {RNS.prettyhexrep(destination.hash)}")
    RNS.log("Press enter to manually send an announcement (Ctrl-C to quit)")
    while True:
        input()  # Waits for the user to press enter
        destination.announce()  # Sends an announcement
        RNS.log(f"Sent announcement from {RNS.prettyhexrep(destination.hash)}")
        if READY_MESSAGE:
            StartMessaging(CLIENT_LINK)  # Starts the message server

# Processes the SNR value of the packet and sends it to the client
def ProcessSNR(packet):
    if hasattr(packet, "snr"):
        snr = packet.snr  # The SNR value of the packet
    else:
        snr = None  # No SNR value

    replyData = CreateCommand(BYTE.SNR, snr)  # Creates a SNR packet
    RNS.Packet(CLIENT_LINK, replyData).send()  # Sends the SNR packet to the client
    RNS.log(f"Sent SNR value: {snr}dB to client")

# Runs when a client connects to the server
def ClientConnected(link):
    global CLIENT_LINK
    RNS.log("Client connected\n")
    link.set_link_closed_callback(ClientDisconnected)  # The function to run when the client disconnects
    link.set_packet_callback(PacketReceived)  # The function to run when a packet is received
    CLIENT_LINK = link  # Stores the link to the client

# Runs when the client disconnects from the server
def ClientDisconnected(link):
    global READY_MESSAGE
    RNS.log("Client disconnected")
    READY_MESSAGE = False  # Resets the flag to indicate that messaging is not ready

##########################################################
#### Client Code #########################################
##########################################################

# Main client function
def Client(destinationHex, configpath):
    try:
        destLength = (RNS.Reticulum.TRUNCATED_HASHLENGTH//8)*2  # The length of the destination hash
        if len(destinationHex) != destLength:
            raise ValueError(f"Must be {destLength} characters")
        destinationHash = bytes.fromhex(destinationHex)  # Converts the destination hash to bytes
    except ValueError:
        RNS.log("Invalid destination hash")
        sys.exit(1)

    RNS.Reticulum(configpath)  # Initialises Reticulum
    if RNS.Transport.has_path(destinationHash):
        return

    RNS.log("Destination is not yet known. Requesting announcement...")
    RNS.Transport.request_path(destinationHash)  # Requests the path to the destination
    while not RNS.Transport.has_path(destinationHash):
        time.sleep(0.1)  # Waits for the path to be established
    serverIdentity = RNS.Identity.recall(destinationHash)  # Gets the identity of the server
    serverDestination = RNS.Destination(
        serverIdentity,
        RNS.Destination.OUT,
        RNS.Destination.SINGLE,
        APP_NAME,
        "PowerTuner"
    )
    link = RNS.Link(serverDestination)  # Creates a link to the server
    link.set_link_established_callback(ServerConnected)
    ClientLoop()

# Main client loop
def ClientLoop():
    global READY_MESSAGE
    serialPort = OpenSerialPort()  # Opens the serial port
    try:
        optimalTX = PowerTuner(serialPort)  # Gets the optimal TX power
        serialPort.close()  # Closes the serial port
        optimalTXData = CreateCommand(BYTE.TX, optimalTX)  # Creates a TX packet
        RNS.Packet(SERVER_LINK, optimalTXData).send()  # Sends the TX packet to the server
        RNS.log(f"Sent optimal TX power: {optimalTX} dBm")
        READY_MESSAGE = True  # Sets the flag to indicate that messaging is ready
        StartMessaging(SERVER_LINK)  # Starts the message client
    except SystemExit:
        RNS.log("Exiting client loop.")
        serialPort.close()
        if SERVER_LINK:
            SERVER_LINK.teardown()  # Tears down the link
        sys.exit(0)
    finally:
        if serialPort:
            serialPort.close()

# Sends empty packets to the server to calculate the optimal TX power
def PowerTuner(serialPort):
    txPower = MAX_TX_POWER  # The initial TX power
    while txPower >= MIN_TX_POWER:
        data = CreateCommand(BYTE.EMPTY)  # Creates an empty packet
        if not (SERVER_LINK and len(data) <= RNS.Link.MDU):
            continue
        previousTX = txPower
        SetTXPower(serialPort, txPower)  # Sets the transmission power
        RNS.Packet(SERVER_LINK, data).send()  # Sends the empty packet to the server
        RNS.log(f"Sent empty packet at {txPower} dBm")
        time.sleep(1.5)

        if SNR_VALUE is not None:
             # Calculates the new TX power
            txPower = BoundryCheck(CalculateTXPower(txPower))
            optimalTX = Stabilisation(txPower, previousTX)  # Stabilises the TX power
            if STABLE_COUNTER != 2:
                continue
            if SNR_VALUE >= SNR_THRESHOLD:
                return optimalTX
            RNS.log("SNR value below threshold. Exiting")
            serialPort.close()
            if SERVER_LINK:
                SERVER_LINK.teardown()
            sys.exit(0)
        else:
            RNS.log("No SNR value received")

# Calculates the new TX power based on the SNR value
def CalculateTXPower(currentTX):
    global ALLOW_DECREMENT
    adjustmentMultiplier = 0.5  # The adjustment multiplier
    snrDifference = SNR_THRESHOLD - SNR_VALUE
    adjustment = adjustmentMultiplier * snrDifference  # The adjustment to the TX power

    trialDecrement = 1  # Small decrement to test a lower TX power
    if -1 <= snrDifference <= 0:
        if ALLOW_DECREMENT:
            adjustment -= trialDecrement  # Decreases the TX power by a small amount
    elif SNR_VALUE < SNR_THRESHOLD:
        adjustment += trialDecrement  # Increases the TX power by a small amount
        ALLOW_DECREMENT = False  # Disables the decrement

    newTXPower = currentTX + int(adjustment)  # Calculates the new TX power

    return newTXPower

# Checks the TX power is within the bounds
def BoundryCheck(txPower):
    if txPower < MIN_TX_POWER:
        return MIN_TX_POWER
    elif txPower >= MAX_TX_POWER:
        return MAX_TX_POWER
    else:
        return txPower

# Stabilises the TX power
def Stabilisation(txPower, previousTX):
    global STABLE_COUNTER
    if txPower == previousTX:
        STABLE_COUNTER += 1
        RNS.log(f"Stable counter: {STABLE_COUNTER}")
    else:
        STABLE_COUNTER = 0
    return txPower

# Runs when the client connects to the server
def ServerConnected(link):
    global SERVER_LINK
    RNS.log("Connected to the server\n")
    link.set_link_closed_callback(ServerDisconnected)  # The function to run when the server disconnects
    link.set_packet_callback(PacketReceived)  # The function to run when a packet is received
    SERVER_LINK = link  # Stores the link to the server

# Runs when the server disconnects from the client
def ServerDisconnected(link):
    if link.teardown_reason == RNS.Link.TIMEOUT:
        RNS.log("The link timed out, exiting now")
    elif link.teardown_reason == RNS.Link.DESTINATION_CLOSED:
        RNS.log("The link was closed by the server, exiting now")
    else:
        RNS.log("Link closed, exiting now")

    RNS.Reticulum.exit_handler()
    time.sleep(1.5)
    sys.exit(0)

# The main code that runs on the server or client
if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser(description="SNR Power Tuner")
        parser.add_argument("-s", "--server", action="store_true", help="Run as server")
        parser.add_argument("destination", nargs="?", type=str, help="Hexadecimal hash of the server destination")
        parser.add_argument("-c", "--config", action="store", type=str, help="Path to an alternative Reticumlum config directory")
        args = parser.parse_args()

        if args.config:
            configarg = args.config  # The path to the config directory
        else:
            configarg = None

        if args.server:
            Server(configarg)  # Runs the server
        else:
            if args.destination:
                Client(args.destination, configarg)  # Runs the client
            else:
                print("\n")
                parser.print_help()
                print("\n")
    except KeyboardInterrupt:
        RNS.log("Exiting")
        RNS.Reticulum.exit_handler()
        sys.exit(0)
