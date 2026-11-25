"""Entry point: start the Qt application and show the main window."""

import sys
from PyQt5 import QtWidgets
from main_window import MainWindow

app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
