#!/usr/bin/env python
import rospy
import numpy as np
import curses  
import time
import atexit
import threading
import sys
from functools import partial
from std_msgs.msg import Float32

SIZE = 64

class AdcScopePrinter(object):
    def __init__(self):
        rospy.init_node('adc_scope_printer', anonymous=True)
        self.values = [0] * SIZE 
        self.values_lock = threading.Lock()
        self.sub = []
        self.done = False
        for i in range(0, SIZE):
            sub = rospy.Subscriber("/adc_scope/analog_input{}".format(i), Float32, 
                    lambda value, i=i: self.on_update(i, value))
            self.sub.append(sub)

    def on_update(self, i, value):
        with self.values_lock:
            self.values[i] = value.data

    def run(self):
        self.t = threading.Thread(target=self.print_values_run)
        self.t.start()
        rospy.spin()

    def stop(self):
        self.done = True
        # self.t.join()

    def print_values_run(self):
        self.stdscr = curses.initscr()  # initialise it
        self.stdscr.nodelay(1)
        asdf = 0
        while not self.done:
            self.print_values()
            asdf +=1 
            ch = self.stdscr.getch()
            if ch > 0:
                self.done = True
            time.sleep(0.2)
        curses.nocbreak()
        self.stdscr.keypad(0)
        self.stdscr.nodelay(0)
        curses.echo()
        curses.endwin()
        sys.exit(0)

    def print_values(self):
        self.stdscr.clear()
        (row, col) =  self.stdscr.getmaxyx()
        max_row = min(row, SIZE/2)
        if col < 100:
            self.stdscr.addstr(1, 0, "terminal too small!")
        else:
            st = ""
            with self.values_lock:
                for i in range(0, max_row-1):
                    st += "% 2d: % .5f      % 2d: % .5f\n" % (i+1, self.values[i], i+1+SIZE/2, self.values[i+SIZE/2])
        self.stdscr.addstr(0, 0, st)
        self.stdscr.refresh()


if __name__ == '__main__':
    adc_scope_printer = AdcScopePrinter()
    adc_scope_printer.run()
