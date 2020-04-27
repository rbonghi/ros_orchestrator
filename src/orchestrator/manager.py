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

import sys
import os
import rospy
import roslaunch
from multiprocessing import Process, Manager, Value, Array
import ctypes
import multiprocessing.sharedctypes as mpsc
from ros_orchestrator.srv import Orchestrator, OrchestratorResponse


class OrchestratorManager:

    def __init__(self):
        self.quite = rospy.get_param("~quite", False)
        # Generate UUID
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # Initialize all launchers
        self.launchers = {}
        for name, launch_data in sorted(rospy.get_param("~orchestrator", []).items()):
            # Initialize lauchers
            if isinstance(launch_data, str):
                launcher = {'file': launch_data}
            elif isinstance(launch_data, list):
                launch = launch_data.get('launch', "")
                args = launch_data.get('args', [])
                for arg in args:
                    rospy.loginfo(arg)
                # Initialize dictionary launcher
                launcher = {'file': launch, 'args': []}
            else:
                rospy.loginfo(arg)
            # Initialzie launcher
            self.launchers[name] = launcher
            rospy.loginfo("Load {name} {launch}".format(name=name, launch=launcher['file']))
        # Initialize orchestrator service server
        rospy.Service('orchestrator', Orchestrator, self.orchestrator_service)

    def launch_process(self, name, nodes):
        # Configure output
        show_summary = False if self.quite else True
        force_log =  self.quite
        # Load launcher
        launcher = self.launchers[name]
        launch_file = launcher['file']
        # print('parent process:', os.getppid())
        # print('process id:', os.getpid())
        rospy.loginfo("[{pid}] ROS launch={launch}".format(pid=os.getpid(), launch=launch_file))
        # Initialize launcher
        launch = roslaunch.parent.ROSLaunchParent(self.uuid, [launch_file], force_log=force_log, show_summary=show_summary)
        launcher['ros_launch'] = launch
        
        rate = rospy.Rate(0.5) # 10hz
        # Run launch file
        launch.start()
        # spin launch
        while launch.pm.is_alive():
            # Call the process monitor
            pm = launch.pm
            if pm is not None:
                active_nodes = pm.get_active_names()
                for idx, node in enumerate(active_nodes):
                    nodes[idx].value = node
            #rospy.loginfo("{}".format(", ".join(active_nodes)))
            # ROS spin one
            launch.spin_once()
            rate.sleep()
        #launch.spin()
        # Launcher close
        rospy.loginfo("Close {launch}".format(launch=launch_file))
        # shutdown the node
        launch.shutdown()

    def _start_process(self, name):
        launcher = self.launchers[name]
        # Check if process is already alive
        if 'process' in launcher:
            if launcher['process'].is_alive():
                return False
        # Initialize process
        #manager = Manager()
        # https://stackoverflow.com/questions/17377426/shared-variable-in-pythons-multiprocessing
        #nodes = Array(ctypes.c_wchar_p, ('', '', ''))
        nodes = [mpsc.RawArray(ctypes.c_char, 30) for _ in range(4)]
        launcher['nodes'] = nodes
        launcher['process'] = Process(target=self.launch_process, args=(name, nodes,))
        # Start process
        launcher['process'].start()
        #launcher['process'].join()
        return True

    def _stop_process(self, name):
        launcher = self.launchers[name]
        # Terminate process if exist
        if 'process' in launcher:
            # shutdown process
            rospy.loginfo("Shutting down {name}".format(name=name))
            launcher['process'].terminate()
        return True

    def _status_process(self, name):
        launcher = self.launchers[name]
        if 'process' in launcher:
            #rospy.loginfo("Status {name}".format(name=name))
            nodes = launcher['nodes']
            nodes = [item.value for item in nodes]
            rospy.loginfo("active nodes {nodes}".format(nodes=str(nodes)))
            #for node in launcher['nodes']:
            #    # nodes = [node.variable for node in launcher['nodes']]
            #    if node is not None:
            #        rospy.loginfo("active nodes {node}".format(node=node))

    def orchestrator_service(self, req):
        # Check if launcher is in list
        launch_status = False
        if req.launch in self.launchers:
            rospy.loginfo("[{status}] launch={launch}".format(launch=req.launch, status=req.status))
            if req.status:
                # Start the launcher
                launch_status = self._start_process(req.launch)
            else:
                # Stop launch file
                launch_status = self._stop_process(req.launch)
        else:
            rospy.logerr("{launch} does not exist!".format(launch=req.launch))
        # return status launcher
        return OrchestratorResponse(launch_status)

    def shutdown(self):
        # Close all launcher active
        for name in self.launchers:
            # Stop process active
            self._stop_process(name)
    
    def status(self):
        """ Read the status of all launchers
        """
        for name in self.launchers:
            # Stop process active
            self._status_process(name)
# EOF
