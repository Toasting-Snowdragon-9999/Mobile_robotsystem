from PySide6.QtWidgets import QVBoxLayout, QLabel, QPushButton, QWidget, QMainWindow, QApplication
from PySide6.QtCore import QTimer, QRunnable, Slot, Signal, QObject, QThreadPool
import sys
import time
import traceback
from typing import Callable, Any, Tuple, Dict, List

class ThreadSignals(QObject):
    '''
    Defines the signals available from a running Second Thread.

    Supported signals are:

    finished
        No data

    error
        tuple (exctype, value, traceback.format_exc() )

    result
        object data returned from processing, anything

    progress
        int indicating % progress

    '''
    finished = Signal()
    error = Signal(tuple)
    result = Signal(object)

class SecondThread(QRunnable):
    runnable_registry = []

    def __init__(self, fn: Callable[..., Any], *args: Tuple[Any, ...], **kwargs: Dict[str, Any], ) -> None:
        super(SecondThread, self).__init__()

        # Store constructor arguments (re-used for processing)
        SecondThread.runnable_registry.append(self)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = ThreadSignals()
        self._is_running = True

    @Slot()
    def run(self) -> None:
        '''
        This is what is being run in a separate thread when we start the SecondThread.
        '''

        # Retrieve args/kwargs here; and fire processing using them
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
            self._is_running = False
        else:
            self.signals.result.emit(result)  # Return the result of the processing
        finally:
            self.signals.finished.emit()  # Done

    def stop(self):
        self._is_running = False

    def __del__(self):
        # Remove the runnable from the registry when it's deleted
        SecondThread.runnable_registry.remove(self)


def list_threads() -> List:
    for runnable in SecondThread.runnable_registry:
        print(f"Runnable: {runnable}")