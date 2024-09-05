import carla


def list_carla_maps(host='localhost', port=2000):
    # Connect to the CARLA server
    client = carla.Client(host, port)
    client.set_timeout(10.0)

    # Get the list of available maps
    world = client.get_world()
    map_names = client.get_available_maps()

    # Print the list of maps
    print("Available maps in CARLA:")
    for name in map_names:
        print(name)


# Run the function with your CARLA server's IP and port
list_carla_maps()
