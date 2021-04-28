#!/usr/bin/env python

import os

import robot_upstart

job = robot_upstart.Job(workspace_setup=os.environ["ROBOT_SETUP"])
job.add(package="jackal_base", filename="launch/base.launch")
job.add(package="jackal_bringup", filename="launch/accessories.launch")

job.install(Provider=robot_upstart.providers.Systemd)
