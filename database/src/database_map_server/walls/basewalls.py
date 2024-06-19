from database_map_server.points.basepoint import BasePoint
from database_map_server.msg import BaseWall as RosWall


class BaseWall:
    def __init__(self, wall_data: dict or RosWall):
        if isinstance(wall_data, dict):
            points = []
            for point in wall_data.get("points"):
                points.append(BasePoint(point))
            self._points = points
            self._name = wall_data.get("name")
        elif isinstance(wall_data, RosWall):
            points = []
            for point in wall_data.points:
                points.append(BasePoint(point))
            self._points = points
            self._name = wall_data.name
        else:
            raise TypeError

    @property
    def points(self):
        return self._points

    @property
    def name(self):
        return self._name

    def to_dict(self):
        points = []
        for point in self.points:
            points.append(point.to_dict())
        result = {"points": points, "name": self.name}
        return result

    def to_ros_wall(self):
        points = []
        for point in self.points:
            points.append(point.to_dict())
        result = RosWall(points=points, name=self.name)
        return result
