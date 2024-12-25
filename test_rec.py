## The following code is a part of render off screen via image queues post at the Carla simulator research blog - https://carlasimblog.wordpress.com/2023/10/18/computer-queues-and-their-use-in-carla-simulator-to-render-large-images-without-image-drops/.
## Feel free to show your support via requested, suggestions and interesting ideas for future research material.


## First, you need to import the necessary modules
import carla
import time
import argparse
import random
import queue
import os
import shutil

IND = 0

def process_windshield_image(image):
    image.save_to_disk(fr'data\test\rgb\{IND}_rgb.png')

def process_right_mirror_image(image):
    image.save_to_disk(fr'data\test\semantic\{IND}_semantic.png')

def process_left_mirror_image(image):
    image.save_to_disk(fr'data\test\depth\{IND}_depth.png')

if __name__ == '__main__':

    ## Connect to the CARLA server
    argparser = argparse.ArgumentParser(
            description='CARLA Sensor tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--sync',
        action='store_false',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_true',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=False)

    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)

    # Load a map and create a new world
    # Getting the world and
    world = client.get_world()
    settings = world.get_settings()
    print(settings)
    settings.no_rendering_mode = True
    settings.synchronous_mode = True
    settings.substepping = True
    ## 0.001 maps us to FPS
    # settings.max_substep_delta_time = 0.001
    settings.fixed_delta_seconds = 0.016
    settings.max_substeps = 10
    world.apply_settings(settings)

    vehicle_blueprint = world.get_blueprint_library().find('vehicle.audi.tt')

    # Define a target velocity in meters per second (m/s)
    # target_velocity = 50.0  # Adjust this value as needed

    # Spawn the vehicle
    random_location = random.choice(world.get_map().get_spawn_points())
    mapp = world.get_map()
    spawn_points = mapp.get_spawn_points()
    spawn_point = (
            random.choice(spawn_points) if spawn_points else carla.Transform()
        )
    
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    vehicle = world.spawn_actor(vehicle_blueprint, spawn_point)
    # vehicle.SetAutopilot(carla.command.FutureActor, True)
    
    
    vehicle.apply_control(carla.VehicleControl())
    vehicle.set_autopilot(True)

    Attachment = carla.AttachmentType


    windshield_queue = queue.Queue()
    left_mirror_queue = queue.Queue()
    right_mirror_queue = queue.Queue()

    _camera_transforms = [
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid, "windshield"),
            (
                carla.Transform(
                    carla.Location(x=0.699997, y=-1.000000, z=1.000000),
                    carla.Rotation(pitch=0.000000, yaw=222.000000, roll=0.000000),
                ),
                Attachment.Rigid, "left"
            ),
            (
                carla.Transform(
                    carla.Location(x=0.699997, y=1.000000, z=1.000000),
                    carla.Rotation(pitch=0.000000, yaw=-222.000000, roll=0.000000),
                ),
                Attachment.Rigid, "right"
            ),
        ]

    camera_bp_rgb = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp_rgb.set_attribute('image_size_x', '1920')
    camera_bp_rgb.set_attribute('image_size_y', '1080')
    camera_bp_rgb.set_attribute('fov', '90')
    camera_transform_rgb_windshield = _camera_transforms[0][0]
    camera_windshield = world.spawn_actor(camera_bp_rgb, camera_transform_rgb_windshield, attachment_type=Attachment.Rigid, attach_to=vehicle)
    camera_windshield.listen(windshield_queue.put)

    camera_bp_rgb = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp_rgb.set_attribute('image_size_x', '1920')
    camera_bp_rgb.set_attribute('image_size_y', '1080')
    camera_bp_rgb.set_attribute('fov', '90')
    camera_transform_rgb_left_mirror = _camera_transforms[1][0]
    camera_left_mirror = world.spawn_actor(camera_bp_rgb, camera_transform_rgb_left_mirror, attachment_type=Attachment.Rigid, attach_to=vehicle)
    camera_left_mirror.listen(left_mirror_queue.put)

    camera_bp_rgb = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp_rgb.set_attribute('image_size_x', '1920')
    camera_bp_rgb.set_attribute('image_size_y', '1080')
    camera_bp_rgb.set_attribute('fov', '90')
    camera_transform_rgb_right_mirror = _camera_transforms[2][0]
    camera_right_mirror = world.spawn_actor(camera_bp_rgb, camera_transform_rgb_right_mirror, attachment_type=Attachment.Rigid, attach_to=vehicle)
    camera_right_mirror.listen(right_mirror_queue.put)

    def create_directories():
        if not os.path.exists("recorded_data"):
            os.makedirs("recorded_data")

        if not os.path.exists("recorded_data/windshield"):
            os.makedirs("recorded_data/windshield")

        if not os.path.exists("recorded_data/left"):
            os.makedirs("recorded_data/left")
        
        if not os.path.exists("recorded_data/right"):
            os.makedirs("recorded_data/right")

    def clear_directories():
        if os.path.exists("recorded_data"):
            shutil.rmtree("recorded_data")

    def process_windshield_image(image):
        image.save_to_disk(f"recorded_data/windshield/{IND}_rgb.png")

    def process_left_mirror_image(image):
        image.save_to_disk(f"recorded_data/left/{IND}_rgb.png")

    def process_right_mirror_image(image):
        image.save_to_disk(f"recorded_data/right/{IND}_rgb.png")

    clear_directories()
    create_directories()

    # timen = time.time()

    while True:
        # print(f"Time: {time.time() - timen}")
        # timen = time.time()
        world.tick()
        windshield_image = windshield_queue.get()
        process_windshield_image(windshield_image)
        left_mirror_image = left_mirror_queue.get()
        process_left_mirror_image(left_mirror_image)
        right_mirror_image = right_mirror_queue.get()
        process_right_mirror_image(right_mirror_image)

        IND += 1