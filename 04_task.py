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
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.objects import DynamicCuboid

from omni.isaac.core.tasks import BaseTask


class FrankaPlaying(BaseTask):
    # NOTE: we only cover here a subset of the task functions that are available,
    # checkout the base class for all the available functions to override.
    # ex: calculate_metrics, is_done..etc.
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
        self._task_achieved = False
        return

    # Here we setup all the assets that we care about in this task.
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self._cube = scene.add(DynamicCuboid(prim_path="/World/random_cube",
                                             name="fancy_cube",
                                             position=np.array(
                                                 [0.3, 0.3, 0.3]),
                                             scale=np.array(
                                                 [0.0515, 0.0515, 0.0515]),
                                             color=np.array([0, 0, 1.0])))
        self._franka = scene.add(Franka(prim_path="/World/Fancy_Franka",
                                        name="fancy_franka"))
        return

    # Information exposed to solve the task is returned from the task through get_observations
    def get_observations(self):
        cube_position, _ = self._cube.get_world_pose()
        current_joint_positions = self._franka.get_joint_positions()
        observations = {
            self._franka.name: {
                "joint_positions": current_joint_positions,
            },
            self._cube.name: {
                "position": cube_position,
                "goal_position": self._goal_position
            }
        }
        return observations

    # Called before each physics step,
    # for instance we can check here if the task was accomplished by
    # changing the color of the cube once its accomplished
    def pre_step(self, control_index, simulation_time):
        cube_position, _ = self._cube.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(self._goal_position - cube_position)) < 0.02:
            # Visual Materials are applied by default to the cube
            # in this case the cube has a visual material of type
            # PreviewSurface, we can set its color once the target is reached.
            self._cube.get_applied_visual_material().set_color(
                color=np.array([0, 1.0, 0]))
            self._task_achieved = True
        return

    # Called after each reset,
    # for instance we can always set the gripper to be opened at the beginning after each reset
    # also we can set the cube's color to be blue
    def post_reset(self):
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_opened_positions)
        self._cube.get_applied_visual_material().set_color(
            color=np.array([0, 0, 1.0]))
        self._task_achieved = False
        return

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
        self.world.add_task(FrankaPlaying(name="franka_playing_task"))

    def setup_post_load(self):
        self.franka = self.world.scene.get_object("fancy_franka")
        self.controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self.franka.gripper,
            robot_articulation=self.franka,
        )
        self.franka.gripper.set_joint_positions(
            self.franka.gripper.joint_opened_positions)
        # Initialize a pick and place controller
        self.controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self.franka.gripper,
            robot_articulation=self.franka,
        )

    def physics_callback(self, dt: float):
        """An example physics callback. It will get invoked every physics step.
        Args:
            dt (float): The time difference between the previous and current function call, in seconds.
        """
        current_observations = self.world.get_observations()
        actions = self.controller.forward(
            picking_position=current_observations["fancy_cube"]["position"],
            placing_position=current_observations["fancy_cube"]["goal_position"],
            current_joint_positions=current_observations["fancy_franka"]["joint_positions"],
        )
        self.franka.apply_action(actions)
        # Only for the pick and place controller, indicating if the state
        # machine reached the final state.
        if self.controller.is_done():
            self.world.pause()

    def reset(self):
        self.cube.set_world_pose(
            position=np.array([0.3, 0.3, 0.3]), orientation=np.array([0, 0, 0, 1])
        )
        self.franka.gripper.set_joint_positions(
            self.franka.gripper.joint_opened_positions)
        self.controller.reset()

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
