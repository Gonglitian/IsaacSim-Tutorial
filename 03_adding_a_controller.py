#!/usr/bin/env python
"""
| File: 0_template_app.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as a template on how to build a clean and simple Isaac Sim based standalone App.
"""

# Imports to start Isaac Sim from this script


from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core import World
import omni.timeline
import carb
import numpy as np
from omni.isaac.core.controllers import BaseController

class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = (
            (2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = (
            (2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)
    
class Template:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the template App and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.world = World()

        # Create a ground plane for the simulation
        self.world.scene.add_default_ground_plane()
        self.setup_scene()
        # Create an example physics callback
        self.world.add_physics_callback(
            'template_physics_callback', self.physics_callback)

        # Create an example render callback
        self.world.add_render_callback(
            'template_render_callback', self.render_callback)

        # Create an example timeline callback
        self.world.add_timeline_callback(
            'template_timeline_callback', self.timeline_callback)

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()
        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def setup_scene(self):
        # you configure a new server with /Isaac folder in it
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
            return
        asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        self.jetbot_robot = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=asset_path,
            )
        )
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        print("Num of degrees of freedom before first reset: " +
              str(self.jetbot_robot.num_dof))  # prints None
        self.robot_controller = CoolController()


    def physics_callback(self, dt: float):
        """An example physics callback. It will get invoked every physics step.
        Args:
            dt (float): The time difference between the previous and current function call, in seconds.
        """
        self.jetbot_robot.apply_action(
            self.robot_controller.forward(command=[0.20, np.pi / 4]))
        ...

    def render_callback(self, data):
        """An example render callback. It will get invoked for every rendered frame.

        Args:
            data: Rendering data.
        """
        carb.log_info("This is a render callback. It is called every frame!")

    def timeline_callback(self, timeline_event):
        """An example timeline callback. It will get invoked every time a timeline event occurs. In this example,
        we will check if the event is for a 'simulation stop'. If so, we will attempt to close the app

        Args:
            timeline_event: A timeline event
        """
        if self.world.is_stopped():
            self.stop_sim = True

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("Template Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():

    # Instantiate the template app
    template_app = Template()

    # Run the application loop
    template_app.run()


if __name__ == "__main__":
    main()
