set(_AMENT_PACKAGE_NAME "gui")
set(gui_VERSION "0.0.0")
set(gui_MAINTAINER "joe <jb5@vt.edu>")
set(gui_BUILD_DEPENDS "rclcpp" "rviz_common" "rviz_rendering" "qtbase5-dev")
set(gui_BUILDTOOL_DEPENDS "ament_cmake")
set(gui_BUILD_EXPORT_DEPENDS "rclcpp" "rviz_common" "rviz_rendering" "qtbase5-dev")
set(gui_BUILDTOOL_EXPORT_DEPENDS )
set(gui_EXEC_DEPENDS "libqt5-core" "libqt5-gui" "libqt5-widgets" "rclcpp" "rviz_common" "rviz_rendering" "qtbase5-dev")
set(gui_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(gui_GROUP_DEPENDS )
set(gui_MEMBER_OF_GROUPS )
set(gui_DEPRECATED "")
set(gui_EXPORT_TAGS)
list(APPEND gui_EXPORT_TAGS "<build_type>ament_cmake</build_type>")