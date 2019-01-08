#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example code to read data from a 9Dof inertial measurement unit (IMU) sensor.

@author: arne.f.meyer@gmail.com
"""

from __future__ import print_function

import numpy as np
from functools import partial
import serial
import threading
import time
import click
import os
import os.path as op
from collections import deque
import copy


class DataThread(threading.Thread):

    def __init__(self,
                 device='/dev/ttyACM0',
                 baudrate=115200,
                 timeout=1.,
                 sync_signal=True):

        super(DataThread, self).__init__()

        self.serial_params = {'device': device,
                              'baudrate': baudrate,
                              'timeout': timeout}
        self.sync_signal = sync_signal

        self.data = []
        self.counter = 0
        self.lock = threading.Lock()
        self.should_exit = False

    def stop(self):

        with self.lock:
            self.should_exit = True

    def run(self):

        with serial.Serial(self.serial_params['device'],
                           baudrate=self.serial_params['baudrate'],
                           timeout=self.serial_params['timeout']) as ser:

            lock = self.lock

            if self.sync_signal:
                # start recording (including sending of synchronization pulses)
                ser.write(b"3\n")
            else:
                ser.write(b"2\n")

            t0 = time.time()
            while True:

                try:
                    line = ser.readline().strip()
                    if len(line) > 0:
                        # each line starts with a ">" and ends with a "<"
                        # as openframeworks serial used in the open-ephys
                        # GUI does not have a proper readline function.
                        # we don't need it here so we can ignore them.
                        line = line[1:-1]

                        values = np.asarray([float(u)
                                             for u in line.split(b",")])

                        if len(values) == 12:
                            # values contains:
                            # status, index, ts, ax, ay, az, gx, gy, gz, mx, my, mz
                            # however, we will throw away the index and status
                            # values
                            with lock:
                                self.data.append(values[2:])
                                self.counter += 1

                        if self.counter % 200 == 0:
                            # this should show the frame rate every 1-2 seconds
                            now = time.time()
                            print("fps:", 200. / (now - t0))
                            t0 = now

                except KeyboardInterrupt:
                    break

                except BaseException:
                    # handle partially transmitted data
                    pass

                with self.lock:
                    if self.should_exit:
                        break

            # stop data acquisition
            ser.write(b"1\n")

    def get_data(self, as_list=False):

        D = None
        with self.lock:

            # X = [x for x in self.data if len(x) == 10]
            X = copy.deepcopy(self.data)
            del self.data[:]

        if len(X) > 0:
            if as_list:
                D = X
            else:
                D = np.vstack(X).T

        return D


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

            if isinstance(x, list):
                self.data.append(x)

            elif isinstance(x, np.ndarray):
                for i in range(x.shape[1]):
                    self.data.append(x[:, i])

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
@click.option('--no-gui', '-n', is_flag=True)
@click.option('--print-to-terminal', '-t', is_flag=True)
def cli(device, baudrate, output, no_gui, print_to_terminal):

    print(device, baudrate, output)

    data_thread = DataThread(device=device,
                             baudrate=baudrate)
    data_thread.start()

    if output is not None:
        write_thread = WriteThread(output)
        write_thread.start()
    else:
        write_thread = None

    if not no_gui:
        # ----- (almost) realtime plotting -----

        from pyqtgraph.Qt import QtGui, QtCore
        import pyqtgraph as pg

        app = QtGui.QApplication([])

        def close_event(data_thread, write_thread):

            data_thread.stop()
            if write_thread is not None:
                write_thread.stop()

            data_thread.join()
            if write_thread is not None:
                write_thread.join()

        app.aboutToQuit.connect(partial(close_event, data_thread, write_thread))

        win = pg.GraphicsWindow()
        win.resize(1000, 800)

        # initial values
        n_samples = 1000
        n_signals = 9
        X = np.zeros((n_signals, n_samples))
        t = np.linspace(0, 1000*int(.005*n_samples), n_samples)

        curves = []
        plots = []
        for i, color in enumerate(['b', 'r', 'y']):
            p = win.addPlot(row=i+1, col=1)
            plots.append(p)
            for _ in range(3):
                c = p.plot(pen=color)
                p.addItem(c)
                c.setPos(0, i*6)
                curves.append(c)

        for p in plots[1:]:
            p.setXLink(plots[0])

        def update(data_thread, write_thread, t, x):

            new_data = data_thread.get_data()

            if write_thread is not None:
                write_thread.append(new_data)

            if new_data is not None and new_data.size > 0:

                M, N = new_data.shape

                x[:, :-N] = x[:, N:]
                x[:, -N:] = new_data[1:, :]

                for j in range(x.shape[0]):
                    curves[i].setData(t, x[j, :])

        timer = QtCore.QTimer()
        timer.timeout.connect(partial(update, data_thread, write_thread, t, X))
        timer.start(25)

        app.exec_()

    else:
        # ----- just save data (and optionally print to terminal) -----
        try:
            while True:

                new_data = data_thread.get_data(as_list=True)

                if new_data is not None:

                    if write_thread is not None:
                        write_thread.append(new_data)

                    if print_to_terminal:
                        print(new_data)

        except KeyboardInterrupt:
            pass

    # clean up
    data_thread.stop()
    data_thread.join()

    if write_thread is not None:
        write_thread.stop()
        write_thread.join()


if __name__ == '__main__':
    cli()
