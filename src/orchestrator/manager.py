# -*- coding: UTF-8 -*-
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
Documentation:
 - http://wiki.ros.org/roslaunch/API%20Usage
 - http://docs.ros.org/melodic/api/roslaunch/html/
 - http://wiki.ros.org/rosout#client_apis
"""


import sys
import os
import rospy
import roslaunch
from diagnostic_msgs.msg import DiagnosticArray
from ros_orchestrator.srv import Orchestrator, OrchestratorResponse
# local Launcher class
from .LauncherProcess import LauncherProcess


class OrchestratorManager:

    def __init__(self, quite=False, rate=0.5):
        # Generate UUID
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        # Diagnostics publisher
        self.diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        # Initialize all launchers
        self.launchers = {}
        for name, launch_data in sorted(rospy.get_param("~orchestrator", []).items()):
            # Initialize lauchers
            launch_file = ""
            args = []
            depend = []
            if isinstance(launch_data, str):
                launch_file = launch_data
            elif isinstance(launch_data, dict):
                launch_file = launch_data.get('launch', "")
                args = ["{key}:={value}".format(key=key, value=value) for key, value in launch_data.get('args', {}).items()]
                depend = launch_data.get('depend', depend)
            else:
                rospy.logerr("{name} Miss launch file information".format(name=name))
                continue
            # Initialzie launcher
            self.launchers[name] = LauncherProcess(uuid, launch_file, name, args=args, depend=depend, quite=quite, rate=rate)
        # Initialize orchestrator service server
        rospy.Service('orchestrator', Orchestrator, self.orchestrator_service)
        # Run all launch file with start
        for name, launch_data in sorted(rospy.get_param("~orchestrator", []).items()):
            if isinstance(launch_data, dict):
                if launch_data.get('start', False):
                    self.startLauncher(name)

    def orchestrator_service(self, req):
        # Check if launcher is in list
        launch_status = False
        name = req.launch
        if name in self.launchers:
            rospy.loginfo("[{status}] launch={name}".format(anme=name, status=req.status))
            # Start or stop launch script
            launch_status = self.startLauncher(name) if req.status else self.stopLauncher(name)
        else:
            rospy.logerr("{name} does not exist!".format(name=name))
        # return status launcher
        return OrchestratorResponse(launch_status)

    def startLauncher(self, name):
        if name not in self.launchers:
            return
        process = self.launchers[name]
        # Check and run all dependencies
        for depend in process.depend:
            # Check if depend launch fils is in list
            if depend in self.launchers:
                dep_process = self.launchers[depend]
                # Chek if is alive
                if not dep_process.alive():
                    rospy.loginfo("{name} depend {depend}".format(name=name, depend=depend))
                    # Run process
                    dep_process.start()
        # Start process active
        return process.start()

    def stopLauncher(self, name):
        if name not in self.launchers:
            return
        process = self.launchers[name]
        # Check and run all dependencies
        for depend in process.depend:
            # Check if depend launch fils is in list
            if depend in self.launchers:
                dep_process = self.launchers[depend]
                # Chek if is alive
                if dep_process.alive():
                    rospy.loginfo("{name} depend {depend}".format(name=name, depend=depend))
                    # Stop process
                    dep_process.stop()
        # Stop process active
        return process.stop()

    def shutdown(self):
        # Close all launcher active
        for name in self.launchers:
            process = self.launchers[name]
            # Stop process active
            process.stop()

    def status(self):
        """ Read the status of all launchers
        """
        # Define Diagnostic array message
        # http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
        arr = DiagnosticArray()
        # Add timestamp
        arr.header.stamp = rospy.Time.now()
        for name in self.launchers:
            # Stop process active
            process = self.launchers[name]
            arr.status += [process.stats()]
        # Publish diagnostic message
        self.diagnostics.publish(arr)
# EOF
