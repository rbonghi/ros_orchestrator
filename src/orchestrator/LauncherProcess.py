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

import rospy
import roslaunch
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
# System
import os
import platform
from multiprocessing import Process, Manager, Value, Array
import ctypes
import multiprocessing.sharedctypes as mpsc

SIZE_NAME = 30

class LauncherProcess(roslaunch.pmon.ProcessListener):
    """
        Launcher process with process monitor
        
        Reference: https://answers.ros.org/question/277789/monitor-remote-nodes-with-roslaunch-api/
    """

    def __init__(self, uuid, launch_file, name, args=[], quite=True, rate=0.5):
        self.uuid = uuid
        self.launch_file = launch_file
        self.name = name
        self.args = args
        self.quite = quite
        self.process = None
        self.rate = rate
        rospy.loginfo("Load {name} {launch} {args}".format(name=name, launch=self.launch_file, args=" ".join(self.args)))
        # Load config
        self.config = roslaunch.config.load_config_default([self.launch_file], None)
        # Load name all nodes
        for node in self.config.nodes:
            rospy.logdebug("{} {} {}".format(node.package, node.type, node.name))

    def process_died(self, name, exit_code):
        rospy.logwarn("{name} died with code {exit_code}".format(name=name, exit_code=exit_code))

    def start(self):
        # Check if process is already alive
        if self.process is not None:
            if self.process.is_alive():
                return False
        # Initialize process
        # https://stackoverflow.com/questions/17377426/shared-variable-in-pythons-multiprocessing
        # https://stackoverflow.com/questions/48727798/shared-memory-array-of-strings-with-multiprocessing
        # Number of nodes
        n_nodes = len(self.config.nodes)
        # Initialize shared data
        self.nodes = [mpsc.RawArray(ctypes.c_char, SIZE_NAME) for _ in range(n_nodes)]
        self.process = Process(target=self.launch_process, args=(self.nodes,))
        # Start process
        self.process.start()
        # TODO check: launcher['process'].join()
        return True

    def stop(self):
        # Terminate process if exist
        if self.process is not None:
            # shutdown process
            self.process.terminate()
        return True

    def stats(self):
        values = [KeyValue("args" , " ".join(self.args))]
        # launcher check
        message = "inactive"
        if self.process is not None:
            if self.process.is_alive():
                # Update message
                message = "running [{pid}]".format(pid=self.process.pid)
                # Update list nodes
                for item in self.nodes:
                    if item.value:
                        status = "active"
                        # Log status nodes in diagnostic
                        values += [KeyValue(str(item.value) , status)]
        # Orchestrator diagnostic
        stats = DiagnosticStatus(name="orchestrator {name}".format(name=self.name),
                                 message=message,
                                 hardware_id=platform.system(),
                                 values=values)
        return stats

    def launch_process(self, nodes):
        # Configure output
        show_summary = False if self.quite else True
        force_log =  self.quite
        rospy.loginfo("[{pid}] ROS launch={launch} {args}".format(pid=os.getpid(), launch=self.launch_file, args=" ".join(self.args)))
        # Initialize launcher
        launch = roslaunch.parent.ROSLaunchParent(self.uuid,
                                                  [(self.launch_file, self.args)], 
                                                  force_log=force_log,
                                                  show_summary=show_summary,
                                                  process_listeners=[self])
        # Rate loop
        rate = rospy.Rate(self.rate)
        # Run launch file
        launch.start()
        # spin launch
        while launch.pm.is_alive():
            # Call the process monitor
            pm = launch.pm
            if pm is not None:
                active_nodes = pm.get_active_names()
                # Update status active nodes
                for idx, node in enumerate(active_nodes):
                    nodes[idx].value = node
            # Show status processes
            # rospy.loginfo("[{pid}] {process}".format(pid=os.getpid(), process=", ".join(active_nodes)))
            # ROS spin one
            launch.spin_once()
            rate.sleep()
        # Launcher close
        rospy.loginfo("Close {launch}".format(launch=self.launch_file))
        # shutdown the node
        launch.shutdown()
# EOF
