class SurveillanceGrid:
    def __init__(self, width, height, obstacles, points_of_interest):
        self.width = width
        self.height = height
        self.obstacles = set(obstacles)
        self.points_of_interest = set(points_of_interest)

    def is_valid_move(self, x, y):
        return (0 <= x < self.width and 0 <= y < self.height and (x, y) not in self.obstacles)

    def get_points_of_interest(self):
        return self.points_of_interest
