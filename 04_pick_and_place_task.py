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
from omni.isaac.core import World
import omni.timeline
import carb
import numpy as np

from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.franka.tasks import PickPlace

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
        self.setup_post_load()
        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def setup_scene(self):
        # you configure a new server with /Isaac folder in it
        self.world.add_task(PickPlace(name="awesome_task"))

    def setup_post_load(self):
        task_params = self.world.get_task("awesome_task").get_params()
        self._franka = self.world.scene.get_object(task_params["robot_name"]["value"])
        self._cube_name = task_params["cube_name"]["value"]
        
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )

    def reset(self):
        self.controller.reset()

    def physics_callback(self, dt: float):
        """An example physics callback. It will get invoked every physics step.
        Args:
            dt (float): The time difference between the previous and current function call, in seconds.
        """
        current_observations = self.world.get_observations()
        actions = self._controller.forward(
            picking_position=current_observations[self._cube_name]["position"],
            placing_position=current_observations[self._cube_name]["target_position"],
            current_joint_positions=current_observations[self._franka.name]["joint_positions"],
        )
        self._franka.apply_action(actions)
        # Only for the pick and place controller, indicating if the state
        # machine reached the final state.
        if self._controller.is_done():
            self.world.pause()

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
