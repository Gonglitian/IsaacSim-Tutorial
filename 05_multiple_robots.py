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

from omni.isaac.franka.tasks import PickPlace
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.tasks import BaseTask
from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.franka.controllers import PickPlaceController

class RobotsPlaying(BaseTask):
    def __init__(
        self,
        name
    ):
        super().__init__(name=name, offset=None)
        self._jetbot_goal_position = np.array([1.3, 0.3, 0])
        # Add task logic to signal to the robots which task is active
        self._task_event = 0
        # Add a subtask of pick and place instead
        # of writing the same task again
        # we just need to add a jetbot and change the positions of the assets and
        # the cube target position
        self._pick_place_task = PickPlace(cube_initial_position=np.array([0.1, 0.3, 0.05]),
                                          target_position=np.array([0.7, -0.3, 0.0515 / 2.0]))
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._pick_place_task.set_up_scene(scene)
        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        self._jetbot = scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
                position=np.array([0, 0.3, 0]))
        )
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(
            pick_place_params["robot_name"]["value"])
        # Changes Franka's default position
        # so that it is set at this position after reset
        self._franka.set_world_pose(position=np.array([1.0, 0, 0]))
        self._franka.set_default_state(position=np.array([1.0, 0, 0]))
        return

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        # Observations needed to drive the jetbot to push the cube
        observations = {
            "task_event": self._task_event,
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position
            }
        }
        observations.update(self._pick_place_task.get_observations())
        return observations

    def get_params(self):
        # To avoid hard coding names..etc.
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {
            "value": self._jetbot.name, "modifiable": False}
        params_representation["franka_name"] = pick_place_params["robot_name"]
        return params_representation

    def pre_step(self, control_index, simulation_time):
        if self._task_event == 0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] - self._jetbot_goal_position[:2])) < 0.04:
                self._task_event += 1
                self._cube_arrive_step_index = control_index
        elif self._task_event == 1:
            # Jetbot has 200 time steps to back off from Franka
            if control_index - self._cube_arrive_step_index == 200:
                self._task_event += 1
        return
    
    def post_reset(self):
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_opened_positions)
        self._task_event = 0
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
        self._world = World()

        # Create a ground plane for the simulation
        self._world.scene.add_default_ground_plane()
        self.setup_scene()
        # Create an example physics callback
        self._world.add_physics_callback(
            'template_physics_callback', self.physics_callback)

        # Create an example render callback
        self._world.add_render_callback(
            'template_render_callback', self.render_callback)

        # Create an example timeline callback
        self._world.add_timeline_callback(
            'template_timeline_callback', self.timeline_callback)

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self._world.reset()
        self.setup_post_load()
        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def setup_scene(self):
        # you configure a new server with /Isaac folder in it
        self._world.add_task(RobotsPlaying(name="awesome_task"))

    def setup_post_load(self):
        task_params = self._world.get_task("awesome_task").get_params()
        # We need franka later to apply to it actions
        self._franka = self._world.scene.get_object(
            task_params["franka_name"]["value"])
        self._jetbot = self._world.scene.get_object(
            task_params["jetbot_name"]["value"])
        # We need the cube later on for the pick place controller
        self._cube_name = task_params["cube_name"]["value"]
        # Add Franka Controller
        self._franka_controller = PickPlaceController(name="pick_place_controller",
                                                      gripper=self._franka.gripper,
                                                      robot_articulation=self._franka)
        self._jetbot = self._world.scene.get_object(
            task_params["jetbot_name"]["value"])
        self._cube_name = task_params["cube_name"]["value"]
        self._jetbot_controller = WheelBasePoseController(name="cool_controller",
                                                          open_loop_wheel_controller=DifferentialController(name="simple_control",
                                                                                                            wheel_radius=0.03, wheel_base=0.1125))

    def physics_callback(self, dt: float):
        current_observations = self._world.get_observations()
        if current_observations["task_event"] == 0:
            self._jetbot.apply_wheel_actions(
                self._jetbot_controller.forward(
                    start_position=current_observations[self._jetbot.name]["position"],
                    start_orientation=current_observations[self._jetbot.name]["orientation"],
                    goal_position=current_observations[self._jetbot.name]["goal_position"]))
        elif current_observations["task_event"] == 1:
            # Go backwards
            self._jetbot.apply_wheel_actions(
                ArticulationAction(joint_velocities=[-8, -8]))
        elif current_observations["task_event"] == 2:
            # Apply zero velocity to override the velocity applied before.
            # Note: target joint positions and target joint velocities will stay active unless changed
            self._jetbot.apply_wheel_actions(
                ArticulationAction(joint_velocities=[0.0, 0.0]))
            # Pick up the block
            actions = self._franka_controller.forward(
                picking_position=current_observations[self._cube_name]["position"],
                placing_position=current_observations[self._cube_name]["target_position"],
                current_joint_positions=current_observations[self._franka.name]["joint_positions"])
            self._franka.apply_action(actions)
        # Pause once the controller is done
        if self._franka_controller.is_done():
            self._world.pause()
        return
    
    def reset(self):
        self._franka_controller.reset()
        self._jetbot_controller.reset()

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
        if self._world.is_stopped():
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
            self._world.step(render=True)

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
