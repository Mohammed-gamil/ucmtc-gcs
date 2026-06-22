"""Compatibility layer that keeps the GCS importable with or without PyQt6.

The workspace used for editing does not ship PyQt6, so this module exposes a
small fallback widget set that mimics the handful of Qt APIs used by the app.
When PyQt6 is installed, the real widgets are re-exported instead.
"""

from __future__ import annotations

import threading
from typing import Any, Callable

QApplication: Any
QGridLayout: Any
QGroupBox: Any
QHBoxLayout: Any
QLabel: Any
QLineEdit: Any
QMainWindow: Any
QPushButton: Any
QTextEdit: Any
QThread: Any
QTimer: Any
QVBoxLayout: Any
QWidget: Any
pyqtSignal: Any

try:  # pragma: no cover - exercised only when PyQt6 is installed.
    from PyQt6.QtCore import QThread, QTimer, pyqtSignal
    from PyQt6.QtWidgets import (
        QApplication,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

    QT_AVAILABLE = True
except ImportError:  # pragma: no cover - used in the workspace.
    QT_AVAILABLE = False

    class _Signal:
        def __init__(self):
            self._slots: list[Callable[..., Any]] = []

        def connect(self, slot: Callable[..., Any]):
            self._slots.append(slot)

        def emit(self, *args: Any, **kwargs: Any):
            for slot in list(self._slots):
                slot(*args, **kwargs)

    class _SignalDescriptor:
        def __init__(self, *args: Any, **kwargs: Any):
            self._storage_name: str | None = None

        def __set_name__(self, owner, name):
            self._storage_name = f"__qt_signal_{name}"

        def __get__(self, instance, owner):
            if instance is None:
                return self
            assert self._storage_name is not None
            signal = instance.__dict__.get(self._storage_name)
            if signal is None:
                signal = _Signal()
                instance.__dict__[self._storage_name] = signal
            return signal

    def pyqtSignal(*args: Any, **kwargs: Any):
        return _SignalDescriptor(*args, **kwargs)

    class QWidget:
        def __init__(self, parent: Any = None):
            self.parent = parent
            self._layout = None
            self._object_name = ""
            self._style_sheet = ""
            self._visible = False
            self._font = None

        def setLayout(self, layout):
            self._layout = layout

        def layout(self):
            return self._layout

        def setObjectName(self, name: str):
            self._object_name = name

        def objectName(self) -> str:
            return self._object_name

        def setStyleSheet(self, style_sheet: str):
            self._style_sheet = style_sheet

        def setFont(self, font):
            self._font = font

        def font(self):
            return self._font

        def show(self):
            self._visible = True

        def hide(self):
            self._visible = False

        def close(self):
            self._visible = False

    class QLabel(QWidget):
        def __init__(self, text: str = "", parent: Any = None):
            super().__init__(parent)
            self._text = text

        def setText(self, text: str):
            self._text = text

        def text(self) -> str:
            return self._text

    class QLineEdit(QWidget):
        def __init__(self, text: str = "", parent: Any = None):
            super().__init__(parent)
            self._text = text
            self._placeholder = ""

        def setText(self, text: str):
            self._text = text

        def text(self) -> str:
            return self._text

        def clear(self):
            self._text = ""

        def setPlaceholderText(self, text: str):
            self._placeholder = text

        def placeholderText(self) -> str:
            return self._placeholder

    class QTextEdit(QWidget):
        def __init__(self, parent: Any = None):
            super().__init__(parent)
            self._text = ""
            self._read_only = False

        def setPlainText(self, text: str):
            self._text = text

        def setText(self, text: str):
            self._text = text

        def toPlainText(self) -> str:
            return self._text

        def append(self, text: str):
            if self._text:
                self._text = f"{self._text}\n{text}"
            else:
                self._text = text

        def setReadOnly(self, read_only: bool):
            self._read_only = read_only

    class QPushButton(QWidget):
        def __init__(self, text: str = "", parent: Any = None):
            super().__init__(parent)
            self._text = text
            self._enabled = True
            self.clicked = _Signal()

        def setText(self, text: str):
            self._text = text

        def text(self) -> str:
            return self._text

        def setEnabled(self, enabled: bool):
            self._enabled = enabled

        def isEnabled(self) -> bool:
            return self._enabled

        def click(self):
            if self._enabled:
                self.clicked.emit()

    class QGroupBox(QWidget):
        def __init__(self, title: str = "", parent: Any = None):
            super().__init__(parent)
            self._title = title

        def setTitle(self, title: str):
            self._title = title

        def title(self) -> str:
            return self._title

    class _BaseLayout:
        def __init__(self):
            self.items: list[tuple[str, Any]] = []
            self._margins = (0, 0, 0, 0)
            self._spacing = 0

        def addWidget(self, widget, *args):
            self.items.append(("widget", widget))

        def addLayout(self, layout, *args):
            self.items.append(("layout", layout))

        def addStretch(self, *args):
            self.items.append(("stretch", args))

        def setContentsMargins(self, left, top, right, bottom):
            self._margins = (left, top, right, bottom)

        def setSpacing(self, spacing):
            self._spacing = spacing

    class QVBoxLayout(_BaseLayout):
        def __init__(self, parent: Any = None):
            super().__init__()

    class QHBoxLayout(_BaseLayout):
        def __init__(self, parent: Any = None):
            super().__init__()

    class QGridLayout(_BaseLayout):
        def __init__(self, parent: Any = None):
            super().__init__()

        def addWidget(self, widget, row, column, row_span=1, column_span=1):
            self.items.append(("widget", widget, row, column, row_span, column_span))

        def addLayout(self, layout, row, column, row_span=1, column_span=1):
            self.items.append(("layout", layout, row, column, row_span, column_span))

    class _StatusBar:
        def __init__(self):
            self._message = ""

        def showMessage(self, message: str):
            self._message = message

        def currentMessage(self) -> str:
            return self._message

    class QMainWindow(QWidget):
        def __init__(self, parent: Any = None):
            super().__init__(parent)
            self._window_title = ""
            self._central_widget = None
            self._status_bar = _StatusBar()
            self._geometry = (0, 0, 0, 0)

        def setWindowTitle(self, title: str):
            self._window_title = title

        def windowTitle(self) -> str:
            return self._window_title

        def setCentralWidget(self, widget):
            self._central_widget = widget

        def centralWidget(self):
            return self._central_widget

        def statusBar(self):
            return self._status_bar

        def resize(self, width: int, height: int):
            self._size = (width, height)

        def setGeometry(self, x: int, y: int, width: int, height: int):
            self._geometry = (x, y, width, height)

    class QApplication:
        _instance = None

        def __init__(self, args: list[str] | None = None):
            QApplication._instance = self
            self.args = args or []

        @classmethod
        def instance(cls):
            return cls._instance

        def exec(self) -> int:
            return 0

        def processEvents(self):
            return None

        def quit(self):
            return None

    class QTimer:
        def __init__(self, parent: Any = None):
            self.parent = parent
            self.timeout = _Signal()
            self._interval_ms = 0
            self._active = False

        def start(self, interval_ms: int):
            self._interval_ms = interval_ms
            self._active = True

        def stop(self):
            self._active = False

        def isActive(self) -> bool:
            return self._active

    class QThread(threading.Thread):
        def __init__(self, parent: Any = None):
            super().__init__(daemon=True)
            self.parent = parent

        def wait(self, timeout: float | None = None):
            self.join(timeout)

        def isRunning(self) -> bool:
            return self.is_alive()

        def quit(self):
            return None


__all__ = [
    "QApplication",
    "QGridLayout",
    "QGroupBox",
    "QHBoxLayout",
    "QLabel",
    "QLineEdit",
    "QMainWindow",
    "QPushButton",
    "QTextEdit",
    "QThread",
    "QTimer",
    "QVBoxLayout",
    "QWidget",
    "QT_AVAILABLE",
    "pyqtSignal",
]
