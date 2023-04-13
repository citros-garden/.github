# CITROS Integration Workflow

This document defines the workflow of integrating a new project into CITROS.

# Table of Contents

* [Creating a New GitHub Project](#creating-a-new-github-project)
* [Pull Request Workflow](#pull-request-workflow)
* [VSCode Devcontainers](#vscode-devcontainers)
* [Creating ROS2 Packages](#creating-ros2-packages)
* [Creating ROS2 Nodes](#creating-ros2-nodes)
* [ROS2 Custom Utilities](#ros2-custom-utilities)
* [ROS2 Parameters](#ros2-parameters)
* [ROS2 Launch](#ros2-launch)
* [Project Setup](#project-setup)
* [ROS2 Build and Run](#ros2-build-and-run)
* [ROS2 Monitoring](#ros2-monitoring)
* [Readme.md](#readmemd)
* [CITROS Deployment](#citros-deployment)


# Creating a New GitHub Project
Use [this template](https://github.com/citros-garden/template) to generate a new GitHub repository. we use lowercase only.
Clone the new repository from GitHub to your local PC, and then reopen the workspace in container under VSCode.

# Pull Request Workflow
Try to keep the main branch as clean as possible without direct commits to it. New features / bugs fixing should be in a dedicated branch. Use the [contribute](contribute.md) document for more information.

# VSCode Devcontainers
Our development environment is based on [devcontainers](https://code.visualstudio.com/docs/devcontainers/containers). All the container configuration located in the `.devcontainer/Dockerfile`, Add necessary linux / pip packages to the Dockerfile and rebuild the workspace.
Don’t install packages / utilities from the command line - write directly to the Dockerfile.

# Creating ROS2 Packages
1. Create an `src` folder under the new repository.
2. `cd` to the repository's `src/` folder.
3. Create a new ROS package with:

        ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs <more_messages_packages>

4. Package name without uppercase letters, use underscores and no dashes.
5. Packages should be separated by functionality.

# Creating ROS2 Nodes
1. Create a new node under `<package>/src/<package>`.
2. Nodes naming without uppercase letters.
3. Use Classes inside the node, try to divide between ROS and functionality.
4. Topics name in lowercase only, divided by
functionality. for Example:\
`/dynamics/position`, `/dynamics/velocity` , `/controller/error`, `/controller/command`

# ROS2 Custom Utilities
Before creating a custom utility, search for a standard one with the same functionality. If there is no standard utility, create a new custom utility in a separate package.\
Add the new custom utility to the `package.xml` file in the required package: `<depend><custom_utility><depend>`

# ROS2 Parameters
CITROS parse [ROS2 parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) and lets the user choose the values. In order to parse the correct parameter file, create a new `param.yaml` file under `<package>/config` folder.
Currently, there is no support in lists/arrays.

# ROS2 Launch
CITROS parse [ROS2 Python Launch files](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) and lets the user choose the required launch file for the simulation. In order to parse the launch files, create a new `<launch>.launch.py` file under `<package>/launch` folder. Try to separate launches by functionality.

# Project Setup
Create an entrypoint and add the launch and config folders to the project paths in the `<package>/setup.py`. 

# ROS2 Build and Run
1. Build the workspace with `colcon build` command in the workspace path:

        colcon build

2. Source the workspace with to see the new packages:

        source install/local_setup.bash

3. Run node with:

         ros2 run <package_name> <node_name> 

4. Run launch file with:

         ros2 launch <package_name> <launch_name>

5. You can configure [VSCode tasks](https://code.visualstudio.com/docs/editor/tasks), it enable run those commands more easily.

# ROS2 Monitoring
After running the simulation, view available topics with:

        ros2 topic list

For fast simulation debugging, use ros2 cli to view the data from topics:

        ros2 topic echo /<topic_name>

For [Foxglove](https://foxglove.dev/) integration, install `rosbridge suite` in the Dockerfile and add `rosbridge_server` to the launch file, then use Websocket data source in Foxglove.

# Readme.md

The project’s readme file should include:

* **General** Information - few lines just to explain the project.
* **Installation** - minimal installation commands.
* **Build** - build commands. Preferred to point to VSCode task instead of CLI commands.
* **Run** - Run commands. Preferred to point to VSCode task instead of CLI commands
* **Develope** - instruction to further develop the simulation.
* **Images / Videos** from Foxglove

The readme should be clean and easy to read,  If it's more complicated than few lines, point to another .md file.

# CITROS Deployment
When the simulation works, create a new Dockerfile for deployment. Use the `.devcontainer/Dockerfile` as a template. It’s recommended to build the ROS2 workspace inside the deployment docker, and not just copy everything with `COPY . .`.
Add a `ros2_entrypoint.sh` with the launch simulation command and add `citros_cli` package to the deployment Dockerfile. Build the Dockerfile and check that the deployment docker is running the simulation.