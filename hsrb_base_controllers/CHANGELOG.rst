^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hsrb_base_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-06-29)
-------------------
* Migration to ROS2 jazzy
* Contributors: Keisuke Takeshita, Shigeo Tsuduki, Yuki Hidaka, Masayuki Masuda

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hsrb_base_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2025-12-04)
-------------------
* Refactor ternary search functions into a single templated version using std::less.
* Add joint velocity constraints considering joint accelerations.
* Separate output and desired velocities
* Change VelocityLimit to Vector3d and simplify processing.
* Fix search conditions for CommandInterface.
* Enabled specification of command_joints to support chainable controllers.
* Replace each package's GetRobotDescription with tmc_utils::ResolveRobotDescription.
* Add base roll control command
* Add namespace parameter
* Contributors: Keisuke Takeshita, Shigeo Tsuduki, Yuki Hidaka

2.3.0 (2025-07-29)
-------------------
* Add controller state topic
* Contributors: Shigeo Tsuduki

2.2.0 (2025-04-22)
-------------------
* Fix hsrb_diag to be available in hsre4p
* Add a parameter to switch the base roll joint to velocity control
* Add follow_distance_trajectory_action
* Fix the issue where the last sampled timestamp is not handled when open_loop_control is set to true.
* Fix the issue where trajectory following fails to function correctly when another action is inserted between the trajectory following actions.
* Fix the issue where holding an unnecessary node pointer caused unstable tests.
* Fixed set_distance command
* Adapt to the package name change
* Fix interpolation method of gripper's trajectory following
* revert test items, revert current_state.positions, modify doxygen comment
* apply gripper improvement of ROS1
* Contributors: Hiroaki Yaguchi, Keisuke Takeshita, Shigeo Tsuduki, Yuki Hidaka

2.1.0 (2024-10-15)
-------------------
* Migration to ROS2 Humble
* Contributors: Hiroaki Yaguchi, Keisuke Takeshita

2.0.0 (2022-10-20)
-------------------
* Migration to ROS2 Foxy
* Contributors: Keisuke Takeshita
