# Apply once at import when running in Docker (MESHCAT_HOST_PORT is set).
from xrobotoolkit_teleop.utils.meshcat_utils import patch_placo_meshcat_host

patch_placo_meshcat_host()
