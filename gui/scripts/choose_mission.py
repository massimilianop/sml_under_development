#!/usr/bin/env python

import sys

from gui.ChooseMission import ChooseMission
from rqt_gui.main import Main

plugin = 'ChooseMission'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
