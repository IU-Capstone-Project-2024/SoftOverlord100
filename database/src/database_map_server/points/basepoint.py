from database_map_server.msg import BasePoint as RosPoint


class BasePoint:
    def __init__(self, point_data: dict or RosPoint):
        if isinstance(point_data, dict):
            self._x = point_data.get("x")
            self._y = point_data.get("y")
            self._name = point_data.get("name")

        elif isinstance(point_data, RosPoint):
            self._x = point_data.x
            self._y = point_data.y
            self._name = point_data.name
        else:
            raise TypeError

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def name(self):
        return self._name

    def to_dict(self):
        result = {"x": self.x, "y": self.y, "name": self.name}
        return result

    def to_ros_point(self):
        result = RosPoint(x=self.x, y=self.y, name=self.name)
        return result
