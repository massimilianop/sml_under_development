#!/usr/bin/env python

import sys

from gui.ChooseJsonable import ChooseJsonable
from rqt_gui.main import Main

plugin = 'ChooseJsonable'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
