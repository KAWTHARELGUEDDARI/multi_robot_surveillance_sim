class MultiRobot:
    def __init__(self, num_robots, start_positions):
        self.robots = [Robot(pos) for pos in start_positions]
        self.paths = [[] for _ in range(num_robots)]

    def set_paths(self, paths):
        self.paths = paths

    def move_step(self):
        for robot in self.robots:
            if robot.path:
                robot.move_step()

    @property
    def positions(self):
        return [robot.position for robot in self.robots]

class Robot:
    def __init__(self, start_pos):
        self.position = start_pos
        self.path = []

    def set_path(self, path):
        self.path = path

    def move_step(self):
        if self.path:
            self.position = self.path.pop(0)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value