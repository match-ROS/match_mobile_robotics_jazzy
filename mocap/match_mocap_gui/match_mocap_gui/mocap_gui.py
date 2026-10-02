#!/usr/bin/env python3
"""Standalone MuR GUI with Qualisys Mocap controls."""

import os
import signal
import sys

from PyQt5 import QtWidgets

from match_mur_gui.base_gui import MurBaseGui
from match_mur_gui.app_icon import configure_gui_icon
from match_mocap_gui.mocap_gui_module import MocapGuiModule


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '62')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    icon = configure_gui_icon(app, "mur-mocap-gui")
    window = MurBaseGui(modules=[MocapGuiModule()], window_title='MuR Mocap GUI')
    window.setWindowIcon(icon)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
