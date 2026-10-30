# Kinova Movement App

A Qt desktop application for preparing and reviewing Kinova Gen3 movement files.

## Requirements

- Python 3.10 or newer
- PySide6 (installed automatically with the project)

## Run

Install the project in editable mode from the project root:

```powershell
python -m pip install -e .
kinova-movement-app
```

The app includes a home screen, a robot connection panel, a movement editor,
and a movement-file preview. Enter the robot's IP address and Kortex credentials
on the home page to open a TCP session (port 10000); use **Close connection**
to end it. Credentials are used for that connection and are not saved.
The editor builds files from manually entered joint-angle waypoints. Added
waypoints and the per-joint transition speeds can be edited directly in the
creation table. Preview playback is available on both the creation and playback
pages, with options to play once, repeat until stopped, set a total play count,
or run to a time limit. Preview playback opens an orbitable animated 3D
seven-joint kinematic approximation and highlights waypoints in the table.
Joint order, angles in degrees, and the zero-angle straight-up pose follow
Kinova's Kortex examples. The preview's forward kinematics use the joint
origins and axes from the [official Gen3 7-DOF URDF](https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-NOVISION_FOR_URDF_ARM_V12.urdf).
The drawn gripper is visually aligned to point upward at the all-zero pose;
this display adjustment does not change arm kinematics. Rendered links are
procedural approximations, not the manufacturer's mesh geometry; live robot
capture is not connected.

Movement files are JSON files containing the movement name, the seven Kinova Gen3
joint names, waypoint angles in degrees, and a `transition_speeds_dps` array with
one seven-joint degrees-per-second speed vector for each position. The final
position's speed vector is used for the transition from the last position back
to the first when looping. The optional `transition_gradients` array stores an
acceleration/deceleration setting and affected joint numbers for each position;
older files without this setting are loaded with ramps disabled. Speed caps are
80°/s for Joints 1–4 and 70°/s for Joints 5–7. For the configured 7-DoF Gen3
Ultra, Joints 1, 3, 5, and 7 accept any finite angle; Joint 2 is limited to
±126°, Joint 4 to ±147°, and Joint 6 to ±117°. Files created by earlier
versions without a loop speed load with a default speed of 30 degrees per
second for the final position.

After saving a movement in Create, it is loaded automatically for playback;
the playback page can also open an existing movement file. Connect from Home
before using **Start robot playback**. Hardware playback supports one pass,
repeat-until-stopped, a play count, and a time limit; **Stop robot** requests a stop at the
current transition. Playback sends feedback-based joint-speed commands, slows
down using the configured deceleration as it approaches each target, and only
advances after measured joint positions remain within the arrival tolerance.
Creation rows can optionally store a
per-transition acceleration/deceleration profile in degrees per second squared,
applied to all joints or a selected subset. These profiles are approximated in
the 3D preview and used for robot playback speed ramps. Robot playback remains
subject to the robot's own limits; it is not a guarantee of motor or operator
safety. Keep the workspace clear and the emergency stop accessible.
Hardware playback has not been verified against a physical robot.

## Test

After installing the project in editable mode:

```powershell
$env:QT_QPA_PLATFORM = "offscreen"
python -m unittest discover -s tests
```

Application code lives in `src/kinova_movement_app/`, with tests in `tests/`.
