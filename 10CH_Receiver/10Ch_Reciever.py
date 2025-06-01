import serial
import time

# Settings
IBUS_BUFFSIZE = 32
IBUS_MAXCHANNELS = 10

# Initialize serial port
ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)  # Adjust '/dev/serial0' if needed
ibus = bytearray(IBUS_BUFFSIZE)
ibus_index = 0
rc_value = [0] * IBUS_MAXCHANNELS

def read_rx():
    global ibus_index

    while ser.in_waiting:
        val = ser.read(1)
        if not val:
            continue

        b = val[0]

        # Look for start of packet
        if ibus_index == 0 and b != 0x20:
            ibus_index = 0
            return
        if ibus_index == 1 and b != 0x40:
            ibus_index = 0
            return

        ibus[ibus_index] = b
        ibus_index += 1

        if ibus_index == IBUS_BUFFSIZE:
            ibus_index = 0

            for i in range(IBUS_MAXCHANNELS):
                low = ibus[2 + i * 2]
                high = ibus[3 + i * 2]
                rc_value[i] = (high << 8) | low

            # Print the channel values
            for idx, ch in enumerate(rc_value):
                print(f"Ch {idx+1}: {ch-1500}", end='  ')
            print()

# Main loop
try:
    while True:
        read_rx()
        # Here you can add robot control code based on rc_value[]!
        time.sleep(0.01)  # Small delay to avoid 100% CPU usage
except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
