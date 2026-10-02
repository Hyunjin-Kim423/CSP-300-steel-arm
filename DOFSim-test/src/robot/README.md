# Robot module (next step)

This folder is intentionally empty for the starter milestone.

Suggested next files:

- `Arm2D.hpp/.cpp` - stores link lengths and joint angles.
- `ForwardKinematics2D.hpp/.cpp` - computes joint/end-effector positions.
- `RobotRenderer.hpp/.cpp` - converts the arm state into renderable geometry.

Keep robot math independent from Vulkan so the kinematics can be unit-tested without opening a window.
