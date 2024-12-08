import os, sys, subprocess
from PySide6.QtCore import QThreadPool

from gui.thread import SecondThread

class RunCpp:
    def __init__(self):
        self.threadpool = QThreadPool()

    def run_exe(self):
        path = os.path.abspath(os.path.join(os.getcwd(), '..', 'cpp', 'build', 'gui_code'))
        print("Executable Path:", path)
        result = subprocess.run([path], capture_output=True, text=True)
        print(result.stdout)  # Print captured stdout)
        return result.stdout

    def run(self):
        print("Running C++ program")
        worker_thread = SecondThread(self.run_exe)
        worker_thread.signals.result.connect(self.result)
        worker_thread.signals.finished.connect(self.thread_complete)
        worker_thread.signals.error.connect(self.thread_error)
        self.threadpool.start(worker_thread)
    
    def result(self, result):
        # print("Result:", result)
        pass

    def thread_complete(self):
        print("Thread complete")
    
    def thread_error(self, error):
        print("Thread error")