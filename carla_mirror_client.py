#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

import glob
import os
import sys

# import pickle
# import socket

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


try:
    sys.path.append(
        glob.glob(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            + "/carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
from time import sleep

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

import carla
from carla import ColorConverter as cc

try:
    import pygame
    from pygame.locals import (
        K_0,
        K_9,
        K_BACKQUOTE,
        K_BACKSPACE,
        K_COMMA,
        K_DOWN,
        K_EQUALS,
        K_ESCAPE,
        K_F1,
        K_LEFT,
        K_MINUS,
        K_PERIOD,
        K_RIGHT,
        K_SLASH,
        K_SPACE,
        K_TAB,
        K_UP,
        KMOD_CTRL,
        KMOD_SHIFT,
        K_a,
        K_b,
        K_c,
        K_d,
        K_f,
        K_g,
        K_h,
        K_i,
        K_j,
        K_k,
        K_l,
        K_m,
        K_n,
        K_o,
        K_p,
        K_q,
        K_r,
        K_s,
        K_v,
        K_w,
        K_x,
        K_z,
        K_KP0,
        K_KP1,
        K_KP2,
        K_KP3,
        K_KP4,
        K_KP5,
        K_KP6,
        K_KP7,
        K_KP8,
        K_KP9,
        K_KP_MINUS,
        K_KP_PLUS,
    )
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame package is installed")

try:
    import numpy as np
except ImportError:
    raise RuntimeError("cannot import numpy, make sure numpy package is installed")


from PIL import Image


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile(".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)")
    name = lambda x: " ".join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match("[A-Z].+", x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = " ".join(actor.type_id.replace("_", ".").title().split(".")[1:])
    return (name[: truncate - 1] + "\u2026") if len(name) > truncate else name


class CarlaMirrorClient:
    def __init__(
        self,
        parent,
        debug=False,
        host="127.0.0.1",
        port=2000,
        autopilot=True,
        res="1920x1200",
        filter="vehicle.*",
        rolename="hero",
        gamma=2.2,
    ):
        self.parent = parent
        self.debug = debug
        self.host = host
        self.port = port
        self.autopilot = autopilot
        self.res = res
        self.filter = filter
        self.rolename = rolename
        self.gamma = gamma

        self.width = int(self.res.split("x")[0])
        self.height = int(self.res.split("x")[1])

        pygame.init()
        pygame.font.init()

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(2.0)
        self.hud = self.HUD(self, self.width, self.height)
        self.client_world = self.client.get_world()
        self.world_object = self.World(self, self.client_world, self.hud)
        self.controller = self.KeyboardControl(self, self.world_object, self.autopilot)
        self.wheel_controller = self.DualControl(
            self, self.world_object, self.autopilot
        )
        self.clock = pygame.time.Clock()

        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.sock.connect(('localhost', 12344))  # Specify the IP and port

    class World(object):
        def __init__(self, parent, carla_world, hud):
            self._parent = parent
            self.world = carla_world
            self.actor_role_name = self._parent.rolename
            try:
                self.map = self._parent.client_world.get_map()
            except RuntimeError as error:
                print("RuntimeError: {}".format(error))
                print("  The server could not send the OpenDRIVE (.xodr) file:")
                print(
                    "  Make sure it exists, has the same name of your town, and is correct."
                )
                sys.exit(1)
            self.hud = hud
            self.player = None
            self.collision_sensor = None
            self.lane_invasion_sensor = None
            self.gnss_sensor = None
            self.imu_sensor = None
            self.radar_sensor = None
            self.camera_manager = None
            self._weather_presets = find_weather_presets()
            self._weather_index = 0
            self._actor_filter = self._parent.filter
            self._gamma = self._parent.gamma
            self.restart()
            self.world.on_tick(hud.on_world_tick)
            self.recording_enabled = False
            self.recording_start = 0
            self.constant_velocity_enabled = False
            self.current_map_layer = 0
            self.map_layer_names = [
                carla.MapLayer.NONE,
                carla.MapLayer.Buildings,
                carla.MapLayer.Decals,
                carla.MapLayer.Foliage,
                carla.MapLayer.Ground,
                carla.MapLayer.ParkedVehicles,
                carla.MapLayer.Particles,
                carla.MapLayer.Props,
                carla.MapLayer.StreetLights,
                carla.MapLayer.Walls,
                carla.MapLayer.All,
            ]

        def restart(self):
            self.player_max_speed = 1.589
            self.player_max_speed_fast = 3.713
            # Keep same camera config if the camera manager exists.
            cam_index = (
                self.camera_manager.index if self.camera_manager is not None else 0
            )
            cam_pos_index = (
                self.camera_manager.transform_index
                if self.camera_manager is not None
                else 0
            )
            # Get a random blueprint.
            # blueprint = random.choice(
            #     self.world.get_blueprint_library().filter(self._actor_filter)
            # )
            blueprint = self.world.get_blueprint_library().find(
                "vehicle.lincoln.mkz_2020"
            )
            blueprint.set_attribute("role_name", self._parent.rolename)
            if blueprint.has_attribute("color"):
                color = random.choice(
                    blueprint.get_attribute("color").recommended_values
                )
                blueprint.set_attribute("color", color)
            if blueprint.has_attribute("driver_id"):
                driver_id = random.choice(
                    blueprint.get_attribute("driver_id").recommended_values
                )
                blueprint.set_attribute("driver_id", driver_id)
            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "true")
            # set the max speed
            if blueprint.has_attribute("speed"):
                self.player_max_speed = float(
                    blueprint.get_attribute("speed").recommended_values[1]
                )
                self.player_max_speed_fast = float(
                    blueprint.get_attribute("speed").recommended_values[2]
                )
            else:
                print("No recommended values for 'speed' attribute")
            # Spawn the player.
            if self.player is not None:
                spawn_point = self.player.get_transform()
                spawn_point.location.z += 2.0
                spawn_point.rotation.roll = 0.0
                spawn_point.rotation.pitch = 0.0
                self.destroy()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            while self.player is None:
                if not self.map.get_spawn_points():
                    print("There are no spawn points available in your map/town.")
                    print("Please add some Vehicle Spawn Point to your UE4 scene.")
                    sys.exit(1)
                spawn_points = self.map.get_spawn_points()
                spawn_point = (
                    random.choice(spawn_points) if spawn_points else carla.Transform()
                )
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.camera_manager = self._parent.CameraManager(
                self.player, self.hud, self._gamma
            )
            self.camera_manager.transform_index = cam_pos_index
            self.camera_manager.set_sensor(cam_index, notify=False)
            actor_type = get_actor_display_name(self.player)
            self.hud.notification(actor_type)

        def next_weather(self, reverse=False):
            self._weather_index += -1 if reverse else 1
            self._weather_index %= len(self._weather_prcarla_clientsets)
            preset = self._weather_presets[self._weather_index]
            self.hud.notification("Weather: %s" % preset[1])
            self.player.get_world().set_weather(preset[0])

        def next_map_layer(self, reverse=False):
            self.current_map_layer += -1 if reverse else 1
            self.current_map_layer %= len(self.map_layer_names)
            selected = self.map_layer_names[self.current_map_layer]
            self.hud.notification("LayerMap selected: %s" % selected)

        def load_map_layer(self, unload=False):
            selected = self.map_layer_names[self.current_map_layer]
            if unload:
                self.hud.notification("Unloading map layer: %s" % selected)
                self.world.unload_map_layer(selected)
            else:
                self.hud.notification("Loading map layer: %s" % selected)
                self.world.load_map_layer(selected)

        def toggle_radar(self):
            if self.radar_sensor is None:
                self.radar_sensor = self.RadarSensor(self.player)
            elif self.radar_sensor.sensor is not None:
                self.radar_sensor.sensor.destroy()
                self.radar_sensor = None

        def tick(self):
            self.hud.tick(self._parent.clock)

        def render(self, display):
            self.camera_manager.render(display)
            self.hud.render(display)

        def destroy_sensors(self):
            self.camera_manager.sensor.destroy()
            self.camera_manager.sensor = None
            self.camera_manager.index = None

        def destroy(self):
            # if self.radar_sensor is not None:
            #     self.toggle_radar()
            sensors = [
                self.camera_manager.sensor,
                # self.collision_sensor.sensor,
                # self.lane_invasion_sensor.sensor,
                # self.gnss_sensor.sensor,
                # self.imu_sensor.sensor,
            ]
            for sensor in sensors:
                if sensor is not None:
                    sensor.stop()
                    sensor.destroy()
            if self.player is not None:
                self.player.destroy()

    # ==============================================================================
    # -- KeyboardControl -----------------------------------------------------------
    # ==============================================================================

    class KeyboardControl(object):
        """Class that handles keyboard input."""

        def __init__(self, parent, world, start_in_autopilot):
            self._parent = parent
            self._carsim_enabled = False
            self._carsim_road = False
            self._chrono_enabled = False
            self._autopilot_enabled = start_in_autopilot
            if isinstance(world.player, carla.Vehicle):
                self._control = carla.VehicleControl()
                self._lights = carla.VehicleLightState.NONE
                world.player.set_autopilot(self._autopilot_enabled)
                world.player.set_light_state(self._lights)
            elif isinstance(world.player, carla.Walker):
                self._control = carla.WalkerControl()
                self._autopilot_enabled = False
                self._rotation = world.player.get_transform().rotation
            else:
                raise NotImplementedError("Actor type not supported")
            self._steer_cache = 0.0
            world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        def parse_events(self, client, world):
            if isinstance(self._control, carla.VehicleControl):
                current_lights = self._lights
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True
                    elif event.key == K_BACKSPACE:
                        if self._autopilot_enabled:
                            world.player.set_autopilot(False)
                            world.restart()
                            world.player.set_autopilot(True)
                        else:
                            world.restart()

                    elif event.key == K_KP7:
                        world.camera_manager.rotate_rear_camera(yaw_increment=-1)
                    elif event.key == K_KP9:
                        world.camera_manager.rotate_rear_camera(yaw_increment=1)
                    elif event.key == K_KP1:
                        world.camera_manager.rotate_rear_camera(pitch_increment=-1)
                    elif event.key == K_KP3:
                        world.camera_manager.rotate_rear_camera(pitch_increment=5)
                    elif event.key == K_KP8:
                        world.camera_manager.move_rear_camera(z_increment=0.01)
                    elif event.key == K_KP2:
                        world.camera_manager.move_rear_camera(z_increment=-0.01)
                    elif event.key == K_KP4:
                        world.camera_manager.move_rear_camera(x_increment=-0.01)
                    elif event.key == K_KP6:
                        world.camera_manager.move_rear_camera(x_increment=0.01)
                    elif event.key == K_KP_MINUS:
                        world.camera_manager.move_rear_camera(y_increment=-0.01)
                    elif event.key == K_KP_PLUS:
                        world.camera_manager.move_rear_camera(y_increment=0.01)
                    elif event.key == K_KP0:
                        world.camera_manager.reset_rear_camera()

                    elif event.key == K_F1:
                        world.hud.toggle_info()
                    elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                        world.next_map_layer(reverse=True)
                    elif event.key == K_v:
                        world.next_map_layer()
                    elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                        world.load_map_layer(unload=True)
                    elif event.key == K_b:
                        world.load_map_layer()
                    elif event.key == K_h or (
                        event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT
                    ):
                        world.hud.help.toggle()
                    elif event.key == K_TAB:
                        world.camera_manager.toggle_camera()
                    elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                        world.next_weather(reverse=True)
                    elif event.key == K_c:
                        world.next_weather()
                    elif event.key == K_g:
                        world.toggle_radar()
                    elif event.key == K_BACKQUOTE:
                        world.camera_manager.next_sensor()
                    elif event.key == K_n:
                        world.camera_manager.next_sensor()
                    elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                        if world.constant_velocity_enabled:
                            world.player.disable_constant_velocity()
                            world.constant_velocity_enabled = False
                            world.hud.notification("Disabled Constant Velocity Mode")
                        else:
                            world.player.enable_constant_velocity(
                                carla.Vector3D(17, 0, 0)
                            )
                            world.constant_velocity_enabled = True
                            world.hud.notification(
                                "Enabled Constant Velocity Mode at 60 km/h"
                            )
                    elif event.key > K_0 and event.key <= K_9:
                        world.camera_manager.set_sensor(event.key - 1 - K_0)
                    elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                        world.camera_manager.toggle_recording()
                    elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                        if world.recording_enabled:
                            client.stop_recorder()
                            world.recording_enabled = False
                            world.hud.notification("Recorder is OFF")
                        else:
                            client.start_recorder("manual_recording.rec")
                            world.recording_enabled = True
                            world.hud.notification("Recorder is ON")
                    elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                        # stop recorder
                        client.stop_recorder()
                        world.recording_enabled = False
                        # work around to fix camera at start of replaying
                        current_index = world.camera_manager.index
                        world.destroy_sensors()
                        # disable autopilot
                        self._autopilot_enabled = False
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification("Replaying file 'manual_recording.rec'")
                        # replayer
                        client.replay_file(
                            "manual_recording.rec", world.recording_start, 0, 0
                        )
                        world.camera_manager.set_sensor(current_index)
                    elif event.key == K_k and (pygame.key.get_mods() & KMOD_CTRL):
                        print("k pressed")
                        if not self._carsim_enabled:
                            self._carsim_enabled = True
                            world.player.enable_carsim()
                        else:
                            self._carsim_enabled = False
                            world.player.restore_physx_physics()
                    elif event.key == K_o and (pygame.key.get_mods() & KMOD_CTRL):
                        print("o pressed")
                        if not self._chrono_enabled:
                            self._chrono_enabled = True
                            vehicle_json = "sedan/vehicle/Sedan_Vehicle.json"
                            powertrain_json = (
                                "sedan/powertrain/Sedan_SimpleMapPowertrain.json"
                            )
                            tire_json = "sedan/tire/Sedan_TMeasyTire.json"
                            base_path = "/home/adas/carla/Build/chrono-install/share/chrono/data/vehicle/"
                            world.player.enable_chrono_physics(
                                5000,
                                0.002,
                                vehicle_json,
                                powertrain_json,
                                tire_json,
                                base_path,
                            )
                        else:
                            self._chrono_enabled = False
                            world.player.restore_physx_physics()
                    elif event.key == K_j and (pygame.key.get_mods() & KMOD_CTRL):
                        self._carsim_road = not self._carsim_road
                        world.player.use_carsim_road(self._carsim_road)
                        print("j pressed, using carsim road =", self._carsim_road)
                    # elif event.key == K_i and (pygame.key.get_mods() & KMOD_CTRL):
                    #     print("i pressed")
                    #     imp = carla.Location(z=50000)
                    #     world.player.add_impulse(imp)
                    elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                        if pygame.key.get_mods() & KMOD_SHIFT:
                            world.recording_start -= 10
                        else:
                            world.recording_start -= 1
                        world.hud.notification(
                            "Recording start time is %d" % (world.recording_start)
                        )
                    elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                        if pygame.key.get_mods() & KMOD_SHIFT:
                            world.recording_start += 10
                        else:
                            world.recording_start += 1
                        world.hud.notification(
                            "Recording start time is %d" % (world.recording_start)
                        )

                    elif event.key == K_f:
                        self._parent.hud.toggle_fullscreen()

                    if isinstance(self._control, carla.VehicleControl):
                        if event.key == K_q:
                            self._control.gear = 1 if self._control.reverse else -1
                        elif event.key == K_m:
                            self._control.manual_gear_shift = (
                                not self._control.manual_gear_shift
                            )
                            self._control.gear = world.player.get_control().gear
                            world.hud.notification(
                                "%s Transmission"
                                % (
                                    "Manual"
                                    if self._control.manual_gear_shift
                                    else "Automatic"
                                )
                            )
                        elif self._control.manual_gear_shift and event.key == K_COMMA:
                            self._control.gear = max(-1, self._control.gear - 1)
                        elif self._control.manual_gear_shift and event.key == K_PERIOD:
                            self._control.gear = self._control.gear + 1
                        elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                            self._autopilot_enabled = not self._autopilot_enabled
                            world.player.set_autopilot(self._autopilot_enabled)
                            world.hud.notification(
                                "Autopilot %s"
                                % ("On" if self._autopilot_enabled else "Off")
                            )
                        elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                            current_lights ^= carla.VehicleLightState.Special1
                        elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                            current_lights ^= carla.VehicleLightState.HighBeam
                        elif event.key == K_l:
                            # Use 'L' key to switch between lights:
                            # closed -> position -> low beam -> fog
                            if not self._lights & carla.VehicleLightState.Position:
                                world.hud.notification("Position lights")
                                current_lights |= carla.VehicleLightState.Position
                            else:
                                world.hud.notification("Low beam lights")
                                current_lights |= carla.VehicleLightState.LowBeam
                            if self._lights & carla.VehicleLightState.LowBeam:
                                world.hud.notification("Fog lights")
                                current_lights |= carla.VehicleLightState.Fog
                            if self._lights & carla.VehicleLightState.Fog:
                                world.hud.notification("Lights off")
                                current_lights ^= carla.VehicleLightState.Position
                                current_lights ^= carla.VehicleLightState.LowBeam
                                current_lights ^= carla.VehicleLightState.Fog
                        elif event.key == K_i:
                            current_lights ^= carla.VehicleLightState.Interior
                        elif event.key == K_z:
                            current_lights ^= carla.VehicleLightState.LeftBlinker
                        elif event.key == K_x:
                            current_lights ^= carla.VehicleLightState.RightBlinker

            if not self._autopilot_enabled:
                if isinstance(self._control, carla.VehicleControl):
                    self._parse_vehicle_keys(
                        pygame.key.get_pressed(), self._parent.clock.get_time()
                    )
                    self._control.reverse = self._control.gear < 0
                    # Set automatic control-related vehicle lights
                    if self._control.brake:
                        current_lights |= carla.VehicleLightState.Brake
                    else:  # Remove the Brake flag
                        current_lights &= ~carla.VehicleLightState.Brake
                    if self._control.reverse:
                        current_lights |= carla.VehicleLightState.Reverse
                    else:  # Remove the Reverse flag
                        current_lights &= ~carla.VehicleLightState.Reverse
                    if (
                        current_lights != self._lights
                    ):  # Change the light state only if necessary
                        self._lights = current_lights
                        world.player.set_light_state(
                            carla.VehicleLightState(self._lights)
                        )
                elif isinstance(self._control, carla.WalkerControl):
                    self._parse_walker_keys(
                        pygame.key.get_pressed(), self.clock.get_time(), world
                    )
                world.player.apply_control(self._control)

        def _parse_vehicle_keys(self, keys, milliseconds):
            if keys[K_UP] or keys[K_w]:
                self._control.throttle = min(self._control.throttle + 0.01, 1)
            else:
                self._control.throttle = 0.0

            if keys[K_DOWN] or keys[K_s]:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._control.brake = 0

            steer_increment = 5e-4 * milliseconds
            if keys[K_LEFT] or keys[K_a]:
                if self._steer_cache > 0:
                    self._steer_cache = 0
                else:
                    self._steer_cache -= steer_increment
            elif keys[K_RIGHT] or keys[K_d]:
                if self._steer_cache < 0:
                    self._steer_cache = 0
                else:
                    self._steer_cache += steer_increment
            else:
                self._steer_cache = 0.0
            self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
            self._control.steer = round(self._steer_cache, 1)
            self._control.hand_brake = keys[K_SPACE]

        def _parse_walker_keys(self, keys, milliseconds, world):
            self._control.speed = 0.0
            if keys[K_DOWN] or keys[K_s]:
                self._control.speed = 0.0
            if keys[K_LEFT] or keys[K_a]:
                self._control.speed = 0.01
                self._rotation.yaw -= 0.08 * milliseconds
            if keys[K_RIGHT] or keys[K_d]:
                self._control.speed = 0.01
                self._rotation.yaw += 0.08 * milliseconds
            if keys[K_UP] or keys[K_w]:
                self._control.speed = (
                    world.player_max_speed_fast
                    if pygame.key.get_mods() & KMOD_SHIFT
                    else world.player_max_speed
                )
            self._control.jump = keys[K_SPACE]
            self._rotation.yaw = round(self._rotation.yaw, 1)
            self._control.direction = self._rotation.get_forward_vector()

        @staticmethod
        def _is_quit_shortcut(key):
            return (key == K_ESCAPE) or (
                key == K_q and pygame.key.get_mods() & KMOD_CTRL
            )

    # ==============================================================================
    # -- DualControl -----------------------------------------------------------
    # ==============================================================================

    class DualControl(object):
        def __init__(self, parent, world, start_in_autopilot):
            self._parent = parent
            self._autopilot_enabled = start_in_autopilot
            if isinstance(world.player, carla.Vehicle):
                self._control = carla.VehicleControl()
                world.player.set_autopilot(self._autopilot_enabled)
            elif isinstance(world.player, carla.Walker):
                self._control = carla.WalkerControl()
                self._autopilot_enabled = False
                self._rotation = world.player.get_transform().rotation
            else:
                raise NotImplementedError("Actor type not supported")
            self._steer_cache = 0.0
            world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

            # initialize steering wheel
            pygame.joystick.init()

            joystick_count = pygame.joystick.get_count()
            if joystick_count > 1:
                raise ValueError("Please Connect Just One Joystick")

            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()

            self._parser = ConfigParser()
            self._parser.read("wheel_config.ini")
            self._steer_idx = int(
                self._parser.get("G29 Racing Wheel", "steering_wheel")
            )
            self._throttle_idx = int(self._parser.get("G29 Racing Wheel", "throttle"))
            self._brake_idx = int(self._parser.get("G29 Racing Wheel", "brake"))
            self._reverse_idx = int(self._parser.get("G29 Racing Wheel", "reverse"))
            self._handbrake_idx = int(self._parser.get("G29 Racing Wheel", "handbrake"))

        def parse_events(self, world, clock):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:
                        world.restart()
                    elif event.button == 1:
                        world.hud.toggle_info()
                    elif event.button == 2:
                        world.camera_manager.toggle_camera()
                    elif event.button == 3:
                        world.next_weather()
                    elif event.button == self._reverse_idx:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.button == 23:
                        world.camera_manager.next_sensor()

                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True
                    elif event.key == K_BACKSPACE:
                        world.restart()
                    elif event.key == K_F1:
                        world.hud.toggle_info()
                    elif event.key == K_h or (
                        event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT
                    ):
                        world.hud.help.toggle()
                    elif event.key == K_TAB:
                        world.camera_manager.toggle_camera()
                    elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                        world.next_weather(reverse=True)
                    elif event.key == K_c:
                        world.next_weather()
                    elif event.key == K_BACKQUOTE:
                        world.camera_manager.next_sensor()
                    elif event.key > K_0 and event.key <= K_9:
                        world.camera_manager.set_sensor(event.key - 1 - K_0)
                    elif event.key == K_r:
                        world.camera_manager.toggle_recording()
                    if isinstance(self._control, carla.VehicleControl):
                        if event.key == K_q:
                            self._control.gear = 1 if self._control.reverse else -1
                        elif event.key == K_m:
                            self._control.manual_gear_shift = (
                                not self._control.manual_gear_shift
                            )
                            self._control.gear = world.player.get_control().gear
                            world.hud.notification(
                                "%s Transmission"
                                % (
                                    "Manual"
                                    if self._control.manual_gear_shift
                                    else "Automatic"
                                )
                            )
                        elif self._control.manual_gear_shift and event.key == K_COMMA:
                            self._control.gear = max(-1, self._control.gear - 1)
                        elif self._control.manual_gear_shift and event.key == K_PERIOD:
                            self._control.gear = self._control.gear + 1
                        elif event.key == K_p:
                            self._autopilot_enabled = not self._autopilot_enabled
                            world.player.set_autopilot(self._autopilot_enabled)
                            world.hud.notification(
                                "Autopilot %s"
                                % ("On" if self._autopilot_enabled else "Off")
                            )

            if not self._autopilot_enabled:
                if isinstance(self._control, carla.VehicleControl):
                    self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                    self._parse_vehicle_wheel()
                    self._control.reverse = self._control.gear < 0
                elif isinstance(self._control, carla.WalkerControl):
                    self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
                world.player.apply_control(self._control)

        def _parse_vehicle_keys(self, keys, milliseconds):
            self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
            steer_increment = 5e-4 * milliseconds
            if keys[K_LEFT] or keys[K_a]:
                self._steer_cache -= steer_increment
            elif keys[K_RIGHT] or keys[K_d]:
                self._steer_cache += steer_increment
            else:
                self._steer_cache = 0.0
            self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
            self._control.steer = round(self._steer_cache, 1)
            self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
            self._control.hand_brake = keys[K_SPACE]

        def _parse_vehicle_wheel(self):
            numAxes = self._joystick.get_numaxes()
            jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
            # print (jsInputs)
            jsButtons = [
                float(self._joystick.get_button(i))
                for i in range(self._joystick.get_numbuttons())
            ]

            # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
            # For the steering, it seems fine as it is
            K1 = 1.0  # 0.55
            steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

            K2 = 1.6  # 1.6
            throttleCmd = (
                K2
                + (2.05 * math.log10(-0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2)
                / 0.92
            )
            if throttleCmd <= 0:
                throttleCmd = 0
            elif throttleCmd > 1:
                throttleCmd = 1

            brakeCmd = (
                1.6
                + (2.05 * math.log10(-0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2)
                / 0.92
            )
            if brakeCmd <= 0:
                brakeCmd = 0
            elif brakeCmd > 1:
                brakeCmd = 1

            self._control.steer = steerCmd
            self._control.brake = brakeCmd
            self._control.throttle = throttleCmd

            # toggle = jsButtons[self._reverse_idx]

            self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

        def _parse_walker_keys(self, keys, milliseconds):
            self._control.speed = 0.0
            if keys[K_DOWN] or keys[K_s]:
                self._control.speed = 0.0
            if keys[K_LEFT] or keys[K_a]:
                self._control.speed = 0.01
                self._rotation.yaw -= 0.08 * milliseconds
            if keys[K_RIGHT] or keys[K_d]:
                self._control.speed = 0.01
                self._rotation.yaw += 0.08 * milliseconds
            if keys[K_UP] or keys[K_w]:
                self._control.speed = (
                    5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
                )
            self._control.jump = keys[K_SPACE]
            self._rotation.yaw = round(self._rotation.yaw, 1)
            self._control.direction = self._rotation.get_forward_vector()

        @staticmethod
        def _is_quit_shortcut(key):
            return (key == K_ESCAPE) or (
                key == K_q and pygame.key.get_mods() & KMOD_CTRL
            )

    # ==============================================================================
    # -- HUD -----------------------------------------------------------------------
    # ==============================================================================

    class HUD(object):
        def __init__(self, parent, width, height):
            self._parent = parent
            self.dim = (width, height)
            font = pygame.font.Font(pygame.font.get_default_font(), 20)
            font_name = "courier" if os.name == "nt" else "mono"
            fonts = [x for x in pygame.font.get_fonts() if font_name in x]
            default_font = "ubuntumono"
            mono = default_font if default_font in fonts else fonts[0]
            mono = pygame.font.match_font(mono)
            self._font_mono = pygame.font.Font(mono, 12 if os.name == "nt" else 14)
            self._notifications = self._parent.FadingText(
                parent, font, (width, 40), (0, height - 40)
            )
            self.help = self._parent.HelpText(pygame.font.Font(mono, 16), width, height)
            self.server_fps = 0
            self.frame = 0
            self.simulation_time = 0
            self._show_info = False
            self._info_text = []
            self._parent.clock = pygame.time.Clock()

            self.fullscreen_toggle = False

        def on_world_tick(self, timestamp):
            self._parent.clock.tick()
            self.server_fps = self._parent.clock.get_fps()
            self.frame = timestamp.frame
            self.simulation_time = timestamp.elapsed_seconds

        def tick(self, world):
            self._notifications.tick(self._parent.world_object, self._parent.clock)
            if not self._show_info:
                return
            t = self._parent.world_object.player.get_transform()
            v = self._parent.world_object.player.get_velocity()
            c = self._parent.world_object.player.get_control()

            vehicles = self._parent.world_object.world.get_actors().filter("vehicle.*")
            self._info_text = [
                "Server:  % 16.0f FPS" % self.server_fps,
                "Client:  % 16.0f FPS" % self._parent.clock.get_fps(),  # hmm
                "",
                "Vehicle: % 20s"
                % get_actor_display_name(self._parent.world_object.player, truncate=20),
                "Map:     % 20s" % self._parent.world_object.map.name.split("/")[-1],
                "Simulation time: % 12s"
                % datetime.timedelta(seconds=int(self.simulation_time)),
                "",
                "Speed:   % 15.0f km/h"
                % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
                "Height:  % 18.0f m" % t.location.z,
                "",
            ]
            if isinstance(c, carla.VehicleControl):
                self._info_text += [
                    ("Throttle:", c.throttle, 0.0, 1.0),
                    ("Steer:", c.steer, -1.0, 1.0),
                    ("Brake:", c.brake, 0.0, 1.0),
                    ("Reverse:", c.reverse),
                    ("Hand brake:", c.hand_brake),
                    ("Manual:", c.manual_gear_shift),
                    "Gear:        %s" % {-1: "R", 0: "N"}.get(c.gear, c.gear),
                ]
            elif isinstance(c, carla.WalkerControl):
                self._info_text += [("Speed:", c.speed, 0.0, 5.556), ("Jump:", c.jump)]

            if len(vehicles) > 1:
                self._info_text += ["Nearby vehicles:"]
                distance = lambda l: math.sqrt(
                    (l.x - t.location.x) ** 2
                    + (l.y - t.location.y) ** 2
                    + (l.z - t.location.z) ** 2
                )
                vehicles = [
                    (distance(x.get_location()), x)
                    for x in vehicles
                    if x.id != self._parent.world_object.player.id
                ]
                for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                    if d > 200.0:
                        break
                    vehicle_type = get_actor_display_name(vehicle, truncate=22)
                    self._info_text.append("% 4dm %s" % (d, vehicle_type))

        def toggle_fullscreen(self):
            if self.fullscreen_toggle:
                self._parent.display = pygame.display.set_mode(
                    (self._parent.width, self._parent.height), pygame.DOUBLEBUF
                )
            else:
                print(self._parent.width, self._parent.height)
                self._parent.display = pygame.display.set_mode(
                    (self._parent.width, self._parent.height),
                    pygame.FULLSCREEN | pygame.DOUBLEBUF,
                )

            pygame.display.flip()

            self.fullscreen_toggle = not self.fullscreen_toggle

        def toggle_info(self):
            self._show_info = not self._show_info

        def notification(self, text, seconds=2.0):
            self._notifications.set_text(text, seconds=seconds)

        def error(self, text):
            self._notifications.set_text("Error: %s" % text, (255, 0, 0))

        def render(self, display):
            if self._show_info:
                info_surface = pygame.Surface((220, self.dim[1]))
                info_surface.set_alpha(100)
                display.blit(info_surface, (0, 0))
                v_offset = 4
                bar_h_offset = 100
                bar_width = 106
                for item in self._info_text:
                    if v_offset + 18 > self.dim[1]:
                        break
                    if isinstance(item, list):
                        if len(item) > 1:
                            points = [
                                (x + 8, v_offset + 8 + (1.0 - y) * 30)
                                for x, y in enumerate(item)
                            ]
                            pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                        item = None
                        v_offset += 18
                    elif isinstance(item, tuple):
                        if isinstance(item[1], bool):
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                            pygame.draw.rect(
                                display, (255, 255, 255), rect, 0 if item[1] else 1
                            )
                        else:
                            rect_border = pygame.Rect(
                                (bar_h_offset, v_offset + 8), (bar_width, 6)
                            )
                            pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                            f = (item[1] - item[2]) / (item[3] - item[2])
                            if item[2] < 0.0:
                                rect = pygame.Rect(
                                    (bar_h_offset + f * (bar_width - 6), v_offset + 8),
                                    (6, 6),
                                )
                            else:
                                rect = pygame.Rect(
                                    (bar_h_offset, v_offset + 8), (f * bar_width, 6)
                                )
                            pygame.draw.rect(display, (255, 255, 255), rect)
                        item = item[0]
                    if item:  # At this point has to be a str.
                        surface = self._font_mono.render(item, True, (255, 255, 255))
                        display.blit(surface, (8, v_offset))
                    v_offset += 18
            self._notifications.render(display)
            self.help.render(display)

    # ==============================================================================
    # -- FadingText ----------------------------------------------------------------
    # ==============================================================================

    class FadingText(object):
        def __init__(self, parent, font, dim, pos):
            self._parent = parent
            self.font = font
            self.dim = dim
            self.pos = pos
            self.seconds_left = 0
            self.surface = pygame.Surface(self.dim)

        def set_text(self, text, color=(255, 255, 255), seconds=2.0):
            text_texture = self.font.render(text, True, color)
            self.surface = pygame.Surface(self.dim)
            self.seconds_left = seconds
            self.surface.fill((0, 0, 0, 0))
            self.surface.blit(text_texture, (10, 11))

        def tick(self, _, clock):
            delta_seconds = 1e-3 * self._parent.clock.get_time()
            self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
            self.surface.set_alpha(500.0 * self.seconds_left)

        def render(self, display):
            display.blit(self.surface, self.pos)

    # ==============================================================================
    # -- HelpText ------------------------------------------------------------------
    # ==============================================================================

    class HelpText(object):
        """Helper class to handle text output using pygame"""

        def __init__(self, font, width, height):
            lines = __doc__.split("\n")
            self.font = font
            self.line_space = 18
            self.dim = (780, len(lines) * self.line_space + 12)
            self.pos = (
                0.5 * width - 0.5 * self.dim[0],
                0.5 * height - 0.5 * self.dim[1],
            )
            self.seconds_left = 0
            self.surface = pygame.Surface(self.dim)
            self.surface.fill((0, 0, 0, 0))
            for n, line in enumerate(lines):
                text_texture = self.font.render(line, True, (255, 255, 255))
                self.surface.blit(text_texture, (22, n * self.line_space))
                self._render = False
            self.surface.set_alpha(220)

        def toggle(self):
            self._render = not self._render

        def render(self, display):
            if self._render:
                display.blit(self.surface, self.pos)

    # ==============================================================================
    # -- CameraManager -------------------------------------------------------------
    # ==============================================================================

    class CameraManager(object):
        def __init__(self, parent_actor, hud, gamma_correction):
            self.sensor = None
            self.surface = None
            self._parent = parent_actor
            self.hud = hud
            self.recording = False
            bound_y = 0.5 + self._parent.bounding_box.extent.y
            Attachment = carla.AttachmentType
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (
                    carla.Transform(
                        carla.Location(x=0.699997, y=-1.000000, z=1.000000),
                        carla.Rotation(pitch=0.000000, yaw=222.000000, roll=0.000000),
                    ),
                    Attachment.Rigid,
                ),
                (
                    carla.Transform(
                        carla.Location(x=0.699997, y=1.000000, z=1.000000),
                        carla.Rotation(pitch=0.000000, yaw=-222.000000, roll=0.000000),
                    ),
                    Attachment.Rigid,
                ),
            ]
            self.transform_index = 1
            self.rear_view_index_left = len(self._camera_transforms) - 1
            self.rear_camera_sensor_left = None
            self.rear_view_array_left = None
            self.rear_view_index_right = len(self._camera_transforms) - 2
            self.rear_camera_sensor_right = None
            self.rear_view_array_right = None
            self.sensors = [
                ["sensor.camera.rgb", cc.Raw, "Camera RGB", {}],
                ["sensor.camera.depth", cc.Raw, "Camera Depth (Raw)", {}],
                ["sensor.camera.depth", cc.Depth, "Camera Depth (Gray Scale)", {}],
                [
                    "sensor.camera.depth",
                    cc.LogarithmicDepth,
                    "Camera Depth (Logarithmic Gray Scale)",
                    {},
                ],
                [
                    "sensor.camera.semantic_segmentation",
                    cc.Raw,
                    "Camera Semantic Segmentation (Raw)",
                    {},
                ],
                [
                    "sensor.camera.semantic_segmentation",
                    cc.CityScapesPalette,
                    "Camera Semantic Segmentation (CityScapes Palette)",
                    {},
                ],
                ["sensor.lidar.ray_cast", None, "Lidar (Ray-Cast)", {"range": "50"}],
                ["sensor.camera.dvs", cc.Raw, "Dynamic Vision Sensor", {}],
                [
                    "sensor.camera.rgb",
                    cc.Raw,
                    "Camera RGB Distorted",
                    {
                        "lens_circle_multiplier": "3.0",
                        "lens_circle_falloff": "3.0",
                        "chromatic_aberration_intensity": "0.5",
                        "chromatic_aberration_offset": "0",
                    },
                ],
            ]
            world = self._parent.get_world()
            bp_library = world.get_blueprint_library()
            for item in self.sensors:
                bp = bp_library.find(item[0])
                if item[0].startswith("sensor.camera"):
                    bp.set_attribute("image_size_x", str(hud.dim[0]))
                    bp.set_attribute("image_size_y", str(hud.dim[1]))
                    if bp.has_attribute("gamma"):
                        bp.set_attribute("gamma", str(gamma_correction))
                    for attr_name, attr_value in item[3].items():
                        bp.set_attribute(attr_name, attr_value)
                elif item[0].startswith("sensor.lidar"):
                    self.lidar_range = 50

                    for attr_name, attr_value in item[3].items():
                        bp.set_attribute(attr_name, attr_value)
                        if attr_name == "range":
                            self.lidar_range = float(attr_value)

                item.append(bp)
            self.index = None

        def get_rear_view_left_as_numpy(self):
            rear_transform = self._camera_transforms[self.rear_view_index_left]

            if (
                hasattr(self, "rear_camera_sensor_left")
                and self.rear_camera_sensor_left is not None
            ):
                return getattr(self, "rear_view_array_left", None)

            bp = self.sensors[0][-1]
            self.rear_camera_sensor_left = self._parent.get_world().spawn_actor(
                bp,
                rear_transform[0],
                attach_to=self._parent,
                attachment_type=rear_transform[1],
            )

            weak_self = weakref.ref(self)

            def parse_rear_view_image_left(weak_self, image):
                self = weak_self()
                if not self:
                    return
                image.convert(self.sensors[0][1])  # Assuming RGB camera for rear view
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                array = array[:, :, :3]  # Get RGB channels
                array = array[:, :, ::-1]  # Convert from BGR to RGB
                self.rear_view_array_left = array

                # print("parsing rear view image left")

                # im = Image.fromarray(array)
                # im.save("left.jpeg")
                if self.recording:
                    image.save_to_disk("left_mirror/%08d.jpg" % image.frame)

                # image.save("left_mirror/%08d.jpg" % image.frame)

            self.rear_camera_sensor_left.listen(
                lambda image: parse_rear_view_image_left(weak_self, image)
            )

            # Return the latest rear view array (ensure it's initialized)
            return getattr(self, "rear_view_array_left", None)

        def get_rear_view_right_as_numpy(self):
            rear_transform = self._camera_transforms[self.rear_view_index_right]

            # print(getattr(self, "rear_view_array_right", None))

            if (
                hasattr(self, "rear_camera_sensor_right")
                and self.rear_camera_sensor_right is not None
            ):
                return getattr(self, "rear_view_array_right", None)

            bp = self.sensors[0][-1]
            self.rear_camera_sensor_right = self._parent.get_world().spawn_actor(
                bp,
                rear_transform[0],
                attach_to=self._parent,
                attachment_type=rear_transform[1],
            )

            weak_self = weakref.ref(self)

            def parse_rear_view_image_right(weak_self, image):
                self = weak_self()
                if not self:
                    return
                image.convert(self.sensors[0][1])
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                array = array[:, :, :3]
                array = array[:, :, ::-1]
                self.rear_view_array_right = array

                if self.recording:
                    image.save_to_disk("right_mirror/%08d.jpg" % image.frame)
                    # im = Image.fromarray(array)
                    # image.save("right_mirror/%08d.jpg" % image.frame)

            self.rear_camera_sensor_right.listen(
                lambda image: parse_rear_view_image_right(weak_self, image)
            )

            return getattr(self, "rear_view_array_right", None)

        def toggle_camera(self):
            self.transform_index = (self.transform_index + 1) % len(
                self._camera_transforms
            )
            self.set_sensor(self.index, notify=False, force_respawn=True)

        def set_sensor(self, index, notify=True, force_respawn=False):
            index = index % len(self.sensors)
            needs_respawn = (
                True
                if self.index is None
                else (
                    force_respawn
                    or (self.sensors[index][2] != self.sensors[self.index][2])
                )
            )
            if needs_respawn:
                if self.sensor is not None:
                    self.sensor.destroy()
                    self.surface = None
                self.sensor = self._parent.get_world().spawn_actor(
                    self.sensors[index][-1],
                    self._camera_transforms[self.transform_index][0],
                    attach_to=self._parent,
                    attachment_type=self._camera_transforms[self.transform_index][1],
                )
                # We need to pass the lambda a weak reference to self to avoid
                # circular reference.
                weak_self = weakref.ref(self)
                self.sensor.listen(lambda image: self._parse_image(weak_self, image))
            if notify:
                self.hud.notification(self.sensors[index][2])
            self.index = index

        def next_sensor(self):
            self.set_sensor(self.index + 1)

        def toggle_recording(self):
            self.recording = not self.recording
            self.hud.notification("Recording %s" % ("On" if self.recording else "Off"))

        def render(self, display):
            if self.surface is not None:
                display.blit(self.surface, (0, 0))

        @staticmethod
        def _parse_image(weak_self, image):
            self = weak_self()
            if not self:
                return
            if self.sensors[self.index][0].startswith("sensor.lidar"):
                points = np.frombuffer(image.raw_data, dtype=np.dtype("f4"))
                points = np.reshape(points, (int(points.shape[0] / 4), 4))
                lidar_data = np.array(points[:, :2])
                lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
                lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
                lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
                lidar_data = lidar_data.astype(np.int32)
                lidar_data = np.reshape(lidar_data, (-1, 2))
                lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
                lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
                lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
                self.surface = pygame.surfarray.make_surface(lidar_img)
            elif self.sensors[self.index][0].startswith("sensor.camera.dvs"):
                # Example of converting the raw_data from a carla.DVSEventArray
                # sensor into a NumPy array and using it as an image
                dvs_events = np.frombuffer(
                    image.raw_data,
                    dtype=np.dtype(
                        [
                            ("x", np.uint16),
                            ("y", np.uint16),
                            ("t", np.int64),
                            ("pol", np.bool),
                        ]
                    ),
                )
                dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
                # Blue is positive, red is negative
                dvs_img[
                    dvs_events[:]["y"], dvs_events[:]["x"], dvs_events[:]["pol"] * 2
                ] = 255
                self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
            else:
                image.convert(self.sensors[self.index][1])
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                array = array[:, :, :3]
                array = array[:, :, ::-1]
                self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

                # print("parsing rear view image")

                # image.save_to_disk("_out/%08d" % image.frame)

            if self.recording:
                image.save_to_disk("_out/%08d.jpg" % image.frame)

    # ==============================================================================
    # -- game_loop() ---------------------------------------------------------------
    # ==============================================================================

    # def send_rear_view(self):
    #     # if self.parent is not None:
    #     rearview_array = self.world_object.camera_manager.get_rear_view_as_numpy()
    #     data = pickle.dumps(rearview_array)  # Serialize the array
    #     self.sock.sendall(data)  # Send serialized data over socket
    #     print("Sent rear view")

    def game_loop(self):
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(200.0)

            original_settings = self.client_world.get_settings()
            settings = self.client_world.get_settings()
            print(settings)
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            self.client_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

            print(self.client_world.get_settings())

            if self.autopilot and not self.client_world.get_settings().synchronous_mode:
                print(
                    "WARNING: You are currently in asynchronous mode and could "
                    "experience some issues with the traffic simulation"
                )

            self.display = pygame.display.set_mode(
                (self.width, self.height), pygame.HWSURFACE | pygame.DOUBLEBUF
            )

            self.client_world.tick()

            while True:
                self.client_world.tick()  # client get_world
                self.clock.tick_busy_loop(60)

                print("Parsing events")

                if self.controller.parse_events(client, self.world_object):
                    return
                    # if self.wheel_controller.parse_events(self.world_object, self.clock):
                    # return

                self.world_object.tick()  # World

                self.world_object.render(self.display)
                pygame.display.flip()

                if True:  # self.parent != None:
                    rearview_array_left = (
                        self.world_object.camera_manager.get_rear_view_left_as_numpy()
                    )
                    # self.parent.update_mirror_frame(rearview_array_left)

                    rearview_array_right = (
                        self.world_object.camera_manager.get_rear_view_right_as_numpy()
                    )
                    # self.parent.update_mirror_frame(rearview_array_right)

                    # print(rearview_array_right)
        finally:

            if original_settings:
                self.client_world.apply_settings(original_settings)

            if self.world_object and self.world_object.recording_enabled:
                client.stop_recorder()

            if self.world_object is not None:
                self.world_object.destroy()

            self.display = pygame.display.set_mode(
                (self.width, self.height), pygame.HWSURFACE | pygame.DOUBLEBUF
            )

            sleep(1)

            pygame.quit()
            sys.exit()

    # ==============================================================================
    # -- main() --------------------------------------------------------------------
    # ==============================================================================


if __name__ == "__main__":
    parent = None
    carla_client = CarlaMirrorClient(parent)

    try:
        carla_client.game_loop()

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")
