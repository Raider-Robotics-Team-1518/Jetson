class FieldData:
    """
    Represents a standard collection of data to be passed between
    processes and to the field management system
    """
    distance = 0
    offset = 0
    target_in_view = False
    is_centered = False
    is_left = False
    is_right = False

    def __init__(self, distance=0, offset=0, target_in_view=False,
                 is_centered=False, is_left=False, is_right=False):
        self.distance = distance
        self.offset = offset
        self.target_in_view = target_in_view
        self.is_centered = is_centered
        self.is_left = is_left
        self.is_right = is_right
