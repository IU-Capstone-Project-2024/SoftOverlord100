import os
from ament_index_python.packages import get_package_share_directory


pack_dir = get_package_share_directory("urdf_dummy")
# os.environ["IGN_GAZEBO_RESOURCE_PATH"] = pack_dir + "/models"

os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
    os.getenv("PWD") + "/simoverlord100/urdf_dummy/models"
)

print(os.getenv("IGN_GAZEBO_RESOURCE_PATH"))
