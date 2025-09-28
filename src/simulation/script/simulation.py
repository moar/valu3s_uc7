import os

from omni.isaac.kit import SimulationApp

CONFIG = {"width": 1280, 
          "height": 720, 
          "sync_loads": True, 
          "headless": True, 
          "renderer": "RayTracedLighting"}
CONFIG = {"width": 1280,
          "height": 720,
          "window_width": 1920,
          "window_height": 1080,
          "headless": True,
          "renderer": "RayTracedLighting"}

# import subprocess

# Define el comando a ejecutar
# comando = ["pkill", "-9", "python3"]

# Ejecuta el comando
# subprocess.run(comando)


# start the omniverse application
kit = SimulationApp(launch_config=CONFIG)

# omniverse import
import omni
import omni.kit.app
from omni.isaac.core.utils.stage import is_stage_loading

# default Livestream settings
kit.set_setting("/app/window/drawMouse", True)
kit.set_setting("/app/livestream/proto", "ws")
kit.set_setting("/app/livestream/websocket/framerate_limit", 120)
kit.set_setting("/ngx/enabled", False)

# default App: Streaming Client from the Omniverse Launcher
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.native")

# enable ROS bridge extension
extension_manager = omni.kit.app.get_app().get_extension_manager()
extension_manager.add_path(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "extensions")), 
                           omni.ext.ExtensionPathType.COLLECTION_USER)
extension_manager.set_extension_enabled_immediate("omni.isaac.ros_bridge", True)
kit.update()
extension_manager.set_extension_enabled_immediate("semu.robotics.ros_bridge", True)
kit.update()
extension_manager.set_extension_enabled_immediate("semu.robotics.tf", True)
kit.update()

# open stage
usd_path =  os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "ros.usd"))
omni.usd.get_context().open_stage(usd_path)
kit.update()  # wait two frames so that stage starts loading
kit.update()

# wait for loading
print("Loading stage: {}".format(usd_path))
while is_stage_loading():
    kit.update()
print("Loading complete")


# simulation/update loop
omni.timeline.get_timeline_interface().play()
while kit.is_running():
    kit.update()  # realtime mode (unspecified step size)
omni.timeline.get_timeline_interface().stop()

kit.close()
