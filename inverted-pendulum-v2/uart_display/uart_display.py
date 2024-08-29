import serial
import struct
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time


# Each data packet will start with this sentinel value.
# While the microprocessor is sending the value out as an int, we
# interpret it as a float so that we can have a consistent way to
# deserialize data over the wire.
sentinel = struct.unpack("f", struct.pack("I", 0xDEADBEEF))[0]

all_values = []
# UART data will be processed in a background thread.
lock = threading.Lock()

# Colors and lables for the final graph.
colors = ["red", "orange", "yellow", "green", "blue", "purple"]
labels = ["encoder", "arm angle", "p comp", "i comp", "d comp", "pid total"]


# get_values returns all the microprocessor data that was delivered
# since the last time get_values was called. This is used
# by the plot drawing code that is running in the main loop.
def get_values():
    global all_values
    lock.acquire()
    vals = []
    new_all_values = []
    for v in all_values:
        vals.append(v[:])
        new_all_values.append([])
    # Clear previously stored values.
    all_values = new_all_values
    lock.release()
    return vals

# add_values adds some values. This is used by the background UART reader.
def add_values(vals):
    lock.acquire()
    while (len(vals) > len(all_values)):
        all_values.append([])
    for i, v in enumerate(vals):
        all_values[i].append(v)
    lock.release()

# nextfloat reads 1 value from the given serialization stream.
def nextfloat(ser):
    return struct.unpack("f", ser.read(4))[0]


# proces_uart reads in UART data. It's meant to be run in a background thread.
def process_uart():
    global current_value
    with serial.Serial('/dev/ttyACM0', 115200) as ser:
        # There's no guarantee that the first value we read will be the start-of-packet
        # sentinel. When forceAlign is true, we keep reading bytes until we find ourselves
        # at the start of a data packet. This happens once on startup, and it may happen
        # later if we find ourselves in some weird state. Maybe we dropped a packet, or maybe
        # the microcontroller was restarted.
        forceAlign = True
        count = 0
        while(True):
            #make sure we're aligned to a 0xDEADBEEF
            if forceAlign:
                forceAlign = False
                print("time to align")
                while(True):
                    if nextfloat(ser) == sentinel:
                        break
                    ser.read()

                # How many numbers until the next sentinel?
                count = 0
                while(True):
                    if nextfloat(ser) == sentinel:
                        break
                    count += 1

            start = time.time()
            nums = []
            for i in range(count):
                nums.append(nextfloat(ser))
            end = time.time()
            add_values(nums)

            # Make sure we have a sentinel value
            curr = nextfloat(ser)
            if curr != sentinel:
                print("misaligned")
                forceAlign = True


# Read UART data in a bg thread.
threading.Thread(target=process_uart).start()

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
xs = []
ys = []

# The X axis is just going to be the total number of data points collected since the start of this execution.
# TODO: Change this to a date/time
counter = 0

# How many data points should we show before they scroll off to the left.
WINDOW_SIZE = 10000

def animate(i, xs, ys):
    global counter

    # Get the data points that were observed since the last time we drew the graph.
    values = get_values()
    if len(values) > 0:
        xs.extend(range(counter, counter+len(values[0])))
        counter += len(values[0])
        while len(xs) < len(values[0]):
            xs.append(counter)
            counter+=1
        while len(ys) < len(values):
            ys.append([])
    for i, vs in enumerate(values):
        ys[i].extend(vs)

    xs = xs[-WINDOW_SIZE:]

    ax.clear()
    small = None
    large = None
    if len(ys) > 0 and len(ys[0]) > 0:
        for vals in ys:
            curr = min(vals)
            if small is None or curr < small:
                small = curr
            if large is None or curr > large:
                large = curr
    for i, _ in enumerate(ys):
        ys[i] = ys[i][-WINDOW_SIZE:]
        ax.plot(xs, ys[i], color=colors[i], label=labels[i])
    if small is None:
        small = 0
    if large is None:
        large = 1

    ax.legend(loc="lower left")
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    #plt.gca().set_ylim([small-20, large+20])
    plt.gca().set_ylim([-100, 100])

    #plt.gca().set_ylim([0, highnum+20])
    #plt.gca().set_ylim([2150, 2200])
   # plt.gca().set_ylim([1500, 2500])


ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=50)
plt.show()



