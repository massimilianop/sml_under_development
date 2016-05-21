#!/usr/bin/env python

import rospy

import missions.missions_database

from utilities import jsonable

# from quad_control.srv import PlotService
import quad_control.srv

import matplotlib
# this is necessary because of threading and plotting
matplotlib.use('Agg')
from matplotlib import pyplot as plt

import matplotlib.backends.backend_pdf

def handle_plot_service(request):

    data_file = request.file_path

    # read mode by default
    read_file = open(data_file).read()

    read_file = read_file.split(jsonable.Jsonable.UNIQUE_STRING)

    read_file.pop(0)

    pdf = matplotlib.backends.backend_pdf.PdfPages(data_file[:-4]+".pdf")

    while len(read_file) >= 1:
        description = read_file.pop(0)

        name, parametric_description = jsonable.Jsonable.inverse_parametric_description(description)

        data = read_file.pop(0)

        print(parametric_description)        

        MissionClass   = missions.missions_database.database[name]
        mission_active = MissionClass.from_string(parametric_description)

        # mission_description = mission_active.description()
        mission_description = mission_active.object_combined_description()

        fig = plt.figure()
        plt.suptitle(mission_description)
        pdf.savefig(fig)
        plt.close(fig)

        for fig in mission_active.plot_from_string(data):
            pdf.savefig(fig)

    pdf.close()
    plt.close("all")

    return quad_control.srv.PlotServiceResponse(True)

if __name__ == '__main__':

    # node will be named plot_node
    rospy.init_node('plot_node', anonymous=True)

    # Service: 
    rospy.Service('PlotService',quad_control.srv.PlotService,handle_plot_service)

    # spin() simply keeps python from exiting
    rospy.spin()