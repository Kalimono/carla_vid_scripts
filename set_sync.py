import carla

client = carla.Client("localhost", 2000)

client_world = client.get_world()

settings = client_world.get_settings()
settings.fixed_delta_seconds = 0.016
settings.synchronous_mode = True # Enables synchronous mode
client_world.apply_settings(settings)