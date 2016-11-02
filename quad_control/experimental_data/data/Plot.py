#!/usr/bin/env python

# need ros to tun code below
import subprocess
subprocess.call("roscore &",shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
import time
time.sleep(1)  # wait a bit to be sure the roscore is really launched

# import missions.missions_database
# MISSIONS_DATABASE = missions.missions_database.database2

import type_uav.type_uav
MISSIONS_DATABASE = type_uav.type_uav.database

from utilities import jsonable

import matplotlib
# this is necessary because of threading and plotting
matplotlib.use('Agg')
from matplotlib import pyplot as plt

import matplotlib.backends.backend_pdf

import pdfkit

from PyPDF2 import PdfFileMerger, PdfFileReader 

# get most recent txt file, and print figures for that file
import os
import glob
DATA_FILE = max(glob.iglob('*.txt'), key=os.path.getctime)
print(DATA_FILE)


# is other file is to be run, use the code below instead
# DATA_FILE = "/home/pedrootao/SML_CODE/src/quad_control/experimental_data/data/_1468688466_untitled_file.txt"

def handle_plot_service():

    data_file = DATA_FILE

    # read mode by default
    read_file = open(data_file).read()

    read_file = read_file.split(jsonable.Jsonable.UNIQUE_STRING)

    # first element in list is "" (empty string)
    read_file.pop(0)

    # remove last 5 lines (because sometimes there are problems when saving this last data) (It commes will NULL symbols)
    read_file[-1] = '\n'.join(map(str, read_file[-1].split('\n')[:-6]))

    merger = PdfFileMerger()

    while len(read_file) >= 1:
        description = read_file.pop(0)

        name, constructing_string = jsonable.Jsonable.inverse_parametric_description(description)

        data = read_file.pop(0)

        MissionClass   = MISSIONS_DATABASE[name]
        mission_active = MissionClass.from_string(constructing_string)

        mission_description = mission_active.object_combined_description()

        pdfkit.from_string(mission_description, data_file[:-4]+".pdf")
        merger.append(PdfFileReader(file(data_file[:-4]+".pdf","rb")))

        fig = plt.figure()
        pdf = matplotlib.backends.backend_pdf.PdfPages(data_file[:-4]+".pdf")

        for fig in mission_active.complete_plot_from_string(data,starting_point=[0]):
            pdf.savefig(fig)

        plt.close("all")
        pdf.close()

        merger.append(PdfFileReader(file(data_file[:-4]+".pdf","rb")))

    merger.write(data_file[:-4]+".pdf")

    return "Done"

handle_plot_service()

# import subprocess
subprocess.call("killall roscore",shell=True)
