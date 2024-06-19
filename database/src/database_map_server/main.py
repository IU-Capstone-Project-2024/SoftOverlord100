# Main file to initialise and launch node database_map_server


import rospy
import std_msgs
from database_route_server.srv import (
    CreateMapRequest,
    GetMapRequest,
    RemoveMapRequest,
    UpdateMapRequest,
)

from database_map_server.maps.basemap import BaseMap

from database_map_server.db_workers.mongodbworker import MongoDBWorker

try:
    # Object of MongoDBWorker to work with DB
    database = MongoDBWorker(
        host=rospy.get_param("db_host"),
        port=rospy.get_param("db_port"),
        username=rospy.get_param("db_username"),
        password=rospy.get_param("db_password"),
        client=rospy.get_param("db_client"),
        collection=rospy.get_param("db_collection"),
    )
except Exception as e:
    rospy.logfatal({"errorMessage": "Cannot connect to MongoDB" + str(e)})
    rospy.signal_shutdown(reason="Cannot connect to MongoDB" + str(e))


pub = rospy.Publisher(name="route/status", data_class=std_msgs.msg.String)


# Work with API call remove_map
# return dict with stucture:
#     Key       |     Type      |                      Value                                         |
# ------------: | :-----------: | :----------------------------------------------------------------- |
#  response     |      bool     |              Is response successfull                               |
#  errorMessage |       str     | String with Error. If response was successfull return empty string |


def callback_remove_map(msg):
    try:
        result = database.remove_map(map_id=msg.id)
    except Exception as e:
        result = {"response": False, "errorMessage": str(e)}
        rospy.logerr(msg, result)
    else:
        rospy.loginfo(msg, result)
    finally:
        return result


# Work with API call create_map
# return dict with stucture:
#     Key       |     Type      |                      Value                                         |
# ------------: | :-----------: | :----------------------------------------------------------------- |
#  response     |      bool     |              Is response successfull                               |
#  errorMessage |       str     | String with Error. If response was successfull return empty string |


def callback_create_map(msg):
    try:
        new_map = BaseMap(msg.map)
        result = database.create_map(new_map)
    except Exception as e:
        result = {"response": False, "errorMessage": str(e)}
        rospy.logerr(msg, result)
    else:
        rospy.loginfo(msg, result)
    finally:
        return result


# Work with API call get_map
# return dict with stucture:
#     Key       |              Type                   |                      Value                                         |
# ------------: | :---------------------------------: | :----------------------------------------------------------------- |
#  response     |              bool                   |              Is response successfull                               |
#  errorMessage |               str                   | String with Error. If response was successfull return empty string |
#   route       | database_map_server/BaseMap         | Requested Map                                                      |


def callback_get_map(msg):
    try:
        result = database.get_map(msg.id)
    except Exception as e:
        result = {"response": False, "errorMessage": str(e)}
        rospy.logerr(msg, result)
    else:
        rospy.loginfo(msg, result)
    finally:
        return result


# work with API call update_map
# return dict with stucture:
#     Key       |              Type                   |                      Value                                         |
# ------------: | :---------------------------------: | :----------------------------------------------------------------- |
#  response     |              bool                   |              Is response successfull                               |
#  errorMessage |               str                   | String with Error. If response was successfull return empty string |


def callback_update_map(msg):
    new_map = BaseMap(msg.map)
    try:
        result = database.update_map(new_map)
    except Exception as e:
        result = {"response": False, "errorMessage": str(e)}
        rospy.logerr(msg, result)
    else:
        rospy.loginfo(msg, result)
    finally:
        return result


# initiate ros node database_route_server
def listen():
    rospy.init_node("database_map_server", anonymous=True)

    # map services
    rospy.Service(
        name="remove_map",
        service_class=RemoveMapRequest,
        handler=callback_remove_map,
    )
    rospy.Service(
        name="create_map",
        service_class=CreateMapRequest,
        handler=callback_create_map,
    )
    rospy.Service(
        name="get_map",
        service_class=GetMapRequest,
        handler=callback_get_map,
    )
    rospy.Service(
        name="update_map",
        service_class=UpdateMapRequest,
        handler=callback_update_map,
    )

    rospy.spin()


if __name__ == "__main__":
    listen()
