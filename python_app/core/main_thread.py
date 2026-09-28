"""
Entrega de trabalho das threads de RX para a thread principal (Qt).

As threads de leitura (serial, Teensy, PeakCAN) só leem bytes/frames e chamam
``relay.post(func, arg)``. O parse e todas as alterações do DeviceState acontecem
na thread da interface, sem corrida com a UI que lê os mesmos objetos.
"""

from PyQt6.QtCore import QObject, pyqtSignal


class MainThreadRelay(QObject):
    _deliver = pyqtSignal(object, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        # AutoConnection: emitido de outra thread vira chamada enfileirada na thread deste objeto
        self._deliver.connect(self._run)

    @staticmethod
    def _run(func, arg):
        func(arg)

    def post(self, func, arg) -> None:
        """Executa func(arg) na thread do relay (a principal); na própria thread, executa já
        (a AutoConnection decide no momento da emissão)."""
        self._deliver.emit(func, arg)
