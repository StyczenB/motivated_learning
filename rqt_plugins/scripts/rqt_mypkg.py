#!/usr/bin/env python3

import sys

from rqt_plugins.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt_plugins'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
