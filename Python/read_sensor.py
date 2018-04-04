#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example code to read data from a 9Dof inertial measurement unit (IMU) sensor.

Created on Thu Mar  8 21:46:17 2018

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

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg


class DataThread(threading.Thread):

    def __init__(self, device='/dev/ttyACM1', baudrate=115200,
                 timeout=1.):

        super(DataThread, self).__init__()

        self.serial = serial.Serial(device, baudrate=baudrate,
                                    timeout=timeout)
        self.data = []
        self.counter = 0
        self.lock = threading.Lock()
        self.should_exit = False

    def stop(self):

        with self.lock:
            self.should_exit = True

    def run(self):

        ser = self.serial
        lock = self.lock

        with lock:
            ser.write("3\n")

        t0 = time.time()
        while True:

            if ser is not None and ser.is_open:

                try:
                    line = ser.readline().strip()
                    if len(line) > 0:

                        values = np.asarray([float(u)
                                             for u in line.split(" ")])

                        if len(values) == 11:
                            # values contains:
                            # status, ts, ax, ay, az, gx, gy, gz, mx, my, mz
                            # however, we will throw away the status value
                            with lock:
                                self.data.append(values[1:])
                                self.counter += 1

                        if self.counter % 200 == 0:
                            # this should show the framerate every 1-2 seconds
                            now = time.time()
                            print("fps:", 200. / (now - t0))
                            t0 = now

                except BaseException:
                    pass

                with self.lock:
                    if self.should_exit:
                        break

        if ser is not None and ser.is_open:
            with lock:
                ser.write("1\n")

    def get_data(self):

        D = None
        with self.lock:
            X = [x for x in self.data if len(x) == 10]
            if len(X) > 0:
                D = np.vstack(X).T
                del self.data[:]

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
                with self.lock:
                    if len(self.data) > 0:
                        line = self.data.popleft()
                        f.write(','.join([str(x) for x in line]) + '\n')

                    if self.should_exit:
                        break


@click.command()
@click.option('--device', '-d', default='/dev/ttyACM0')
@click.option('--baudrate', '-b', default=115200)
@click.option('--output', '-o', default=None)
def cli(device, baudrate, output):

    print(device, baudrate, output)

    # initial values
    n_samples = 1000
    n_signals = 9
    X = np.zeros((n_signals, n_samples))
    t = np.linspace(0, 1000*int(.005*n_samples), n_samples)

    data_thread = DataThread(device=device, baudrate=baudrate)
    data_thread.start()

    if output is not None:
        write_thread = WriteThread(output)
        write_thread.start()
    else:
        write_thread = None

    # ----- (almost) realtime plotting -----
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

    curves = []
    plots = []
    for i, color in enumerate(['b', 'r', 'y']):
        p = win.addPlot(row=i+1, col=1)
        plots.append(p)
        for j in range(3):
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

            for i in range(x.shape[0]):
                curves[i].setData(t, x[i, :])

    timer = QtCore.QTimer()
    timer.timeout.connect(partial(update, data_thread, write_thread, t, X))
    timer.start(25)

    app.exec_()


if __name__ == '__main__':
    cli()
