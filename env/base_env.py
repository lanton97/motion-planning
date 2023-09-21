
# A base environment class for constructing scenes for the motion
# planning algorithms to plan n
class BaseEnv():
    # Initialize a list of obstacles(walls, moving items, etc.) and controllable objects(Vehicles)
    def __init__(self):
        pass

    # Check if a given motion will result in a collision with an obstacle
    def is_collision(self, new_positions):
        raise NotImplementedError

    # Update the positions of vehicles or any moving obstacles in the environment
    # The base is simply a pass as we assume only static obstacles
    def update(self):
        pass

