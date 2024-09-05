import carla

def get_collision_volume(blueprint_name):
    # Connect to CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and blueprint library
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Find the specific blueprint
    blueprint = blueprint_library.find(blueprint_name)

    if blueprint is None:
        print(f"Blueprint {blueprint_name} not found.")
        return None

    # Print all attributes of the blueprint
    print("Attributes of the blueprint:")
    for attr in blueprint:
        print(f" - {attr.id}: {attr.type}")

    # Check for bounding box or collision-related attributes
    if blueprint.has_tag('bounding_box'):
        bbox = blueprint.get_attribute('bounding_box')
        # Calculate volume: V = width * height * depth
        extent = bbox.extent
        volume = extent.x * 2 * extent.y * 2 * extent.z * 2
        return volume
    else:
        print(f"No bounding box attribute found for {blueprint_name}.")
        return None

# Usage
blueprint_name = 'static.prop.pergola'
collision_volume = get_collision_volume(blueprint_name)
if collision_volume:
    print(f"The collision volume for {blueprint_name} is: {collision_volume:.2f} cubic meters.")
else:
    print("Could not calculate the collision volume.")
