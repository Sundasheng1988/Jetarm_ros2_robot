cmake_minimum_required(VERSION 3.8)
project(place_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Detect Python version for site-packages path (relative to install prefix)
execute_process(
  COMMAND python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
# Match the Python install path used by rosidl_generate_interfaces.
# On Ubuntu 22.04 / Python 3.10, rosidl installs generated code to
#   ${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/place_manager/srv/
# so we must install our hand-written modules to the SAME top-level
# place_manager/ directory, otherwise PYTHONPATH order may hide our code.
set(PYTHON_SITE "local/lib/python${PYTHON_VERSION}/dist-packages")

# ── ROS2 interface generation ──────────────────────────────────────────────
# Generates Python code for all 4 srv files so that
#   from place_manager.srv import SavePlace, DeletePlace, ListPlaces, GetPlace
# works in the node code, and ros2 service call can resolve the type.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePlace.srv"
  "srv/DeletePlace.srv"
  "srv/ListPlaces.srv"
  "srv/GetPlace.srv"
  "srv/GotoPlace.srv"
  DEPENDENCIES geometry_msgs
)

# ── Python package installation ────────────────────────────────────────────
# Installs hand-written place_manager/ Python module into site-packages.
# NOTE: rosidl_generate_interfaces internally handles its generated Python
#       code installation, so we must NOT call ament_python_install_package
#       again (it creates conflicting custom targets).  Instead we use a
#       plain install(DIRECTORY ...) to the detected site-packages dir.
install(DIRECTORY place_manager/
  DESTINATION ${PYTHON_SITE}/place_manager
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# ── Entry point scripts for ros2 run ───────────────────────────────────────
# Hybrid ament_cmake_python does not auto-generate entry_points from setup.py,
# so we install wrapper scripts manually.
install(PROGRAMS
  scripts/place_manager_node
  scripts/goto_place_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch files ───────────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Config template ────────────────────────────────────────────────────────
# The config/places.yaml in the package is a template only.
# The default runtime file is at ~/ros2_ws/config/places.yaml,
# overridable via launch parameter places_file.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Resource marker for ament_index ────────────────────────────────────────
install(FILES resource/place_manager
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
