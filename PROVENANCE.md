# Source provenance

Developed by PyLoT Robotics from work in `PyLoT-Robotics/pylot_robocup2025`,
commit `55960a6c7bcdaff870a593dedbbb46a7bbeee375`.

The state machine and its characterization tests were adapted from
`utils/mft_teleop/mft_teleop/core.py` and `utils/mft_teleop/test/test_core.py`.
Machine paths, branding, and the fixed linear-speed ceiling were generalized.
The new ROS adapter uses standard messages, configurable joint profiles, and
isolated default output topics instead of private robot interfaces.

No models, recordings, credentials, or third-party SDKs are bundled.
This standalone project is distributed under Apache-2.0; see LICENSE.
