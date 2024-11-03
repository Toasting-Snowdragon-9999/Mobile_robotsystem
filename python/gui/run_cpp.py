import os, sys, subprocess
from PySide6.QtCore import QThreadPool

from gui.thread import SecondThread

class RunCpp:
    def __init__(self):
        self.threadpool = QThreadPool()

    def run_exe(self):
        path = "..//cpp//build//mobile_system"
        result = subprocess.run(path, capture_output=True, text=True)
        print("Output:", result.stdout)
        print("Errors:", result.stderr)
        return result.stdout

    def run(self):
        print("Running C++ program")
        print(os.getcwd())
        worker_thread = SecondThread(self.run_exe)
        worker_thread.signals.result.connect(self.result)
        worker_thread.signals.finished.connect(self.thread_complete)
        worker_thread.signals.error.connect(self.thread_error)
        self.threadpool.start(worker_thread)
    
    def result(self, result):
        print("Result:", result)
    
    def thread_complete(self):
        print("Thread complete")
    
    def thread_error(self, error):
        print("Thread error")