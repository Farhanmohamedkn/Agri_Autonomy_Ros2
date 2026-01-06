class LawnMowerPlanner:
    """
    Lawn-mower coverage planner for rectangular agricultural fields.
    Output is a continuous path in meters (local map frame).
    """

    def __init__(self, field_length, field_width, row_spacing):
        self.field_length = field_length
        self.field_width = field_width
        self.row_spacing = row_spacing

    def generate_path(self):
        """
        Generate a list of (x, y) path points.
        """
        path = []
        y = 0.0
        direction = 1

        while y <= self.field_width:
            if direction == 1:
                path.append((0.0, y))
                path.append((self.field_length, y))
            else:
                path.append((self.field_length, y))
                path.append((0.0, y))

            y += self.row_spacing
            direction *= -1

        return path
