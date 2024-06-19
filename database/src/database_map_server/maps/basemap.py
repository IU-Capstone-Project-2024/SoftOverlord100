from database_map_server.points.basepoint import BasePoint
from database_map_server.walls.basewalls import BaseWall
from database_map_server.msg import BaseMap as RosMap


class BaseMap:
    def __init__(self, map_data: dict or RosMap):
        if isinstance(map_data, dict):
            points = []
            for point in map_data.get("points"):
                points.append(BasePoint(point))
            self._points = points

            walls = []
            for wall in map_data.get("walls"):
                walls.append(BaseWall(wall))
            self._walls = walls

            self._map_id = str(map_data.get("_id"))

            self._name = map_data.get("name")

            self._preview_url = map_data.get("preview_url")

        elif isinstance(map_data, RosMap):
            points = []
            for point in map_data.points:
                points.append(BasePoint(point))
            self._points = points

            walls = []
            for wall in map_data.walls:
                walls.append(BaseWall(wall))
            self._walls = walls

            self._map_id = map_data.id

            self._name = map_data.name

            self._preview_url = map_data.previewURL
        else:
            raise TypeError

    @property
    def points(self):
        return self._points

    @property
    def walls(self):
        return self._walls

    @property
    def id(self):
        return self._map_id

    @property
    def name(self):
        return self._name

    @property
    def preview_url(self):
        return self._preview_url

    def to_dict(self):
        points = []
        for point in self.points:
            points.append(point.to_dict())

        walls = []
        for wall in self.walls:
            walls.append(wall.to_dict())

        result = {
            "_id": self.id,
            "preview_url": self.preview_url,
            "name": self.name,
            "walls": walls,
            "points": points,
        }
        return result

    def to_ros_map(self):
        points = []
        for point in self.points:
            points.append(point.to_dict())

        walls = []
        for wall in self.walls:
            walls.append(wall.to_dict())

        result = RosMap(
            id=self.id,
            previewUrl=self.preview_url,
            name=self.name,
            walls=walls,
            points=points,
        )
        return result
