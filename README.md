This repo lets you run a PX4 SITL multicopter (x500) inside Gazebo (Ignition/GZ) using a custom environment built from an RTAB-Map reconstruction. We convert an RTAB-Map .ply point cloud to a renderable mesh (.obj) and reference it inside a self-contained SDF world—no Fuel downloads required.

What you get

A minimal SDF world that includes:

A basic sun (directional light)

A cheap ground plane (no network dependency)

Your OBJ mesh as static scenery

A PX4 x500 drone spawn point

Scripts/notes to convert .ply → .obj

Steps to run:

Gazebo only (for quick world checks)

PX4 SITL + Gazebo (for flight simulations)
