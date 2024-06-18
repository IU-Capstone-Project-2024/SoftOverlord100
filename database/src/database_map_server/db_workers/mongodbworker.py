import pymongo
from bson import ObjectId

from database.src.database_map_server.maps.basemap import BaseMap
from database_map_server.db_workers.dbworker import DBWorker


# Class to work with mongoDB, inherits from DBWorker
class MongoDBWorker(DBWorker):
    # init object, connect to database and check ping
    def __init__(self, host, port, username, password, client, collection):
        self.client = pymongo.MongoClient(
            host=host,
            port=port,
            username=username,
            password=password,
            serverSelectionTimeoutMS=2000,
        )
        self.db = self.client[client]
        self.db_collection = self.db[collection]
        self.posts = self.db.posts
        self.client.server_info()

    # Remove map with id map_id from mongoDB
    # return dict with stucture:
    #     Key       |     Type      |                      Value                                         |
    # ------------: | :-----------: | :----------------------------------------------------------------- |
    #  response     |      bool     |              Is response successfull                               |
    #  errorMessage |       str     | String with Error. If response was successfull return empty string |

    def remove_map(self, map_id: str) -> dict:
        map_exists = list(self.db_collection.find({"_id": ObjectId(map_id)}))
        if not map_exists:
            return {
                "response": False,
                "errorMessage": "Map with this ID does not exist in DB",
            }

        self.db_collection.delete_one({"_id": ObjectId(map_id)})
        return {"response": True, "errorMessage": ""}

    # Save new map in DB
    # return dict with stucture:
    #     Key       |     Type      |                      Value                                         |
    # ------------: | :-----------: | :----------------------------------------------------------------- |
    #  response     |      bool     |              Is response successfull                               |
    #  errorMessage |       str     | String with Error. If response was successfull return empty string |

    def create_map(self, map: BaseMap) -> dict:
        # TODO: check that data is good

        # TODO: check if map alredy in DB
        map_exists = list(
            self.db_collection.find(
                {
                    # check fields
                }
            )
        )
        if map_exists:
            return {"response": False, "errorMessage": "Map already exists in DB"}

        new_map = map.to_dict()
        self.db_collection.insert_one(new_map)
        return {"response": True, "errorMessage": ""}

    # Return requested map by its ID
    # return dict with stucture:
    #     Key       |              Type                   |                      Value                                         |
    # ------------: | :---------------------------------: | :----------------------------------------------------------------- |
    #  response     |              bool                   |              Is response successfull                               |
    #  errorMessage |               str                   | String with Error. If response was successfull return empty string |
    #   route       | database_map_server.msg.BaseMap     | Requested Map                                                      |

    def get_map(self, map_id: str) -> dict:
        if not map_id:
            return {"response": False, "errorMessage": "Empty Map ID"}

        db_map = list(
            self.db_collection.find({"_id": ObjectId(map_id), "data_type": "route"})
        )

        if len(db_map) == 0:
            return {
                "response": False,
                "errorMessage": "Map with this ID does not exist in DB",
            }

        map = BaseMap(db_map[0])

        return {"response": True, "errorMessage": "", "route": map.to_ros_map()}

    # Update map in mongoDB
    # return dict with stucture:
    #     Key       |     Type      |                      Value                                         |
    # ------------: | :-----------: | :----------------------------------------------------------------- |
    #  response     |      bool     |              Is response successfull                               |
    #  errorMessage |       str     | String with Error. If response was successfull return empty string |

    def update_map(self, map: BaseMap) -> dict:
        map_exists = list(
            self.db_collection.find({"_id": ObjectId(map.map_id), "data_type": "route"})
        )
        if not map_exists:
            return {
                "response": False,
                "errorMessage": "Map with this ID does not exist in DB",
            }

        # TODO: check correctness of data

        dict_route = map.to_dict()
        self.db_collection.update_one(
            {"_id": ObjectId(map.route_id)},
            {
                "$set": {
                    # TODO: set parameters
                }
            },
            upsert=False,
        )

        return {"response": True, "errorMessage": ""}

    # TODO: make verification of password
    # Check if given password hash correspond to real password hash
    # return dict with stucture:
    #     Key       |     Type      |                      Value                                         |
    # ------------: | :-----------: | :----------------------------------------------------------------- |
    #  response     |      bool     |              Is response successfull                               |
    #  errorMessage |       str     | String with Error. If response was successfull return empty string |

    def verify_password(self, passwordHash):
        pass
