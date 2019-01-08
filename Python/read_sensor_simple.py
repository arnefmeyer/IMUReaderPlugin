#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Very simple example code to read data from a 9Dof inertial measurement unit (IMU) sensor.

@author: arne.f.meyer@gmail.com
"""

from __future__ import print_function

import click
import serial
import time
import os.path as op
import numpy as np
import threading
from collections import deque
import traceback


class WriteThread(threading.Thread):

    def __init__(self, output):

        assert output is not None

        super(WriteThread, self).__init__()

        if isinstance(output, str):

            output = op.expanduser(output)

            if op.isdir(output):
                # directory
                if not op.exists(output):
                    os.makedirs(output)
                of = op.join(output, 'imu_data.csv')
            else:
                # file
                output_dir = op.split(output)[0]
                if not op.exists(output_dir):
                    os.makedirs(output_dir)
                of = output
        else:
            # stream etc
            of = output

        self.output = output
        self.file = of
        self.lock = threading.Lock()
        self.should_exit = False
        self.data = deque()

    def append(self, x):

        with self.lock:
            self.data.append(x)

    def stop(self):

        with self.lock:
            self.should_exit = True

    def run(self):

        with open(self.file, 'w') as f:

            while True:

                line = None
                with self.lock:
                    if len(self.data) > 0:
                        line = self.data.popleft()

                if line is not None:
                    f.write(','.join([str(x) for x in line]) + '\n')
                else:
                    time.sleep(.01)

                with self.lock:
                    if self.should_exit:
                        break


@click.command()
@click.option('--device', '-d', default='/dev/ttyACM0')
@click.option('--baudrate', '-b', default=115200)
@click.option('--output', '-o', default=None)
@click.option('--print-to-terminal', '-t', is_flag=True)
@click.option('--skip-lines', '-S', is_flag=True)
@click.option('--sync-signal', '-s', is_flag=True)
def cli(device, baudrate, output, print_to_terminal, skip_lines, sync_signal):

    with serial.Serial(device,
                       baudrate=baudrate,
                       timeout=1.) as ser:

        if output is not None:
            write_thread = WriteThread(output)
            write_thread.start()
            time.sleep(.1)
        else:
            write_thread = None

        ser.reset_input_buffer()

        if sync_signal:
            # start recording (including sending of synchronization pulses)
            ser.write(b"3\n")
        else:
            ser.write(b"2\n")

        t0 = time.time()
        counter = 0
        while True:

            try:
                line = ser.readline().strip()
                if len(line) > 0:
                    # each line starts with a ">" and ends with a "<"
                    # as openframeworks serial used in the open-ephys
                    # GUI does not have a proper readline function.
                    # we don't need it here so we can ignore them.
                    line = line[1:-1]

                    values = [float(u) for u in line.split(b",")]

                    if len(values) == 12:
                        # values contains:
                        # status, index, ts, ax, ay, az, gx, gy, gz, mx, my, mz
                        # however, we will throw away the index and status
                        # values
                        if print_to_terminal:
                            if not skip_lines or (counter % 5 == 0):
                                print("{:9.0f} {:+2.6f} {:+2.6f} {:+2.6f} {:+4.2f} {:+4.2f} {:+4.2f} {:+3.2f} {:+3.2f} {:+3.2f}".format(
                                    *values[2:]))

                        if write_thread is not None:
                            write_thread.append(values[2:])

                        counter += 1

                if counter % 200 == 0:
                    # this should show the frame rate every 1-2 seconds
                    now = time.time()
                    print("fps:", 200. / (now - t0))
                    t0 = now

            except KeyboardInterrupt:
                break

            except BaseException:
                # print information about potential problems, e.g., partially transmitted data
                traceback.print_exc()

        # stop data acquisition
        ser.write(b"1\n")

        if write_thread is not None:
            write_thread.stop()
            write_thread.join()


if __name__ == '__main__':
    cli()
