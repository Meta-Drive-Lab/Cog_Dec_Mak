import sys
from PyQt5.QtCore import QObject, pyqtSignal

class StdoutRedirector(QObject):
    # 定义一个信号，用于将输出发送到 UI
    print_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        # self.old_stdout = sys.stdout  # 保存原始的 sys.stdout

    def write(self, text):
        # 将输出发送到信号
        self.print_signal.emit(text)
        # 同时保留原始输出（可选）
        # self.old_stdout.write(text)

    # def flush(self):
        # 需要实现 flush 方法
        # self.old_stdout.flush()