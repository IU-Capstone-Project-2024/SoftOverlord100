from abc import ABCMeta, abstractmethod
from database.src.database_map_server.maps.basemap import BaseMap


# Abstract class to work with DB


class DBWorker(metaclass=ABCMeta):

    @abstractmethod
    def create_map(self, map_data: str) -> dict:
        pass

    @abstractmethod
    def get_map(self, map_id: str) -> dict:
        pass

    @abstractmethod
    def remove_map(self, map_id: str) -> dict:
        pass

    @abstractmethod
    def update_map(self, new_map: BaseMap) -> dict:
        pass