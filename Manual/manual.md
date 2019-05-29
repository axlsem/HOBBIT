# [General](#general)
This tool can be used to create, edit and manage programs on the HOBBIT robot platform as well as any other ROS-based robots. It is implemented as a web-based application using [Google's Blockly framework](https://developers.google.com/blockly/).

## Prerequisites
To run the tool the following software must be installed:
- ROS
- Node.js (v8.11+)

The current implementation of this tool is build an version 1.0.0 of Google's Blockly and uses it as a static dependency. This means that any update provided by the Blockly community is not applied to the tool, but the user does not need to take care to install Blockly. It is already included - see section [UI Directory](#ui-directory).
 
## Download
The tool is available at the [V4R GitLab](https://rgit.acin.tuwien.ac.at/) within the *hobbit* project. The full ROS package is also located in the *hobbit_blockly* folder of the project and there provides the source code within the *data* folder. It must be downloaded to the robot which should be controlled by the tool.

## Configuration
Configurations can be made by setting the following enviroment variables before starting the server:
* PORT: the port to which the tool is listening (default: 8080)
* SRCDIR: path where executable *.py* file is temporary stored (see [UI Directory](#ui-directory))

Enviroment variables can be set like this:
```
# UNIX (method 1)
PORT=3000

# UNIX (method 2)
export PORT=3000

# Windows
set PORT=3000

# Windows PowerShell
$env:PORT=3000
```

## Starting the server
The server for this tool runs directly on the robot and automatically starts when the `startup.launch` launch file of the `hobbit_blockly` ROS package is executed via the `roslaunch` command. This basically starts a bash file, which then starts the server via the `node app.js` command. After that the tool can be used accessed with a browser via the ip address of the robot and the port the server is listening to, e.g. `10.0.0.105:8080`.

### Remarks
- Be sure that the port the tool listens to is forwareded by the router
- Port 8080 is the default setting, it can be changed by setting the corresponding enviroment variable before starting the server
- Any changes to the server code/configuration during runtime will not be applied, it's necessary to start the server again (hit CTRL+C to stop it).

# [Demo Management](#demo-management)
The landing page shown when starting the tool gives an overview of all created demos saved on the robot. It is possible to run, edit and delete them directly from the graphical interface - just click on the corresponding button. Furthermore the main page provides links to [create a new demo](#create-a-new-demo) and to the [block configurator](#create-custom-blocks).

![Demo Management Page](./images/DemoManagementFrontend.png)

# [Create a demo](#create-a-demo)
Blockly is a library that adds a visual code editor to web and Android apps. It consists of a toolbox, from where the progamming blocks can be dragged to the workspace where they are connected. For a quick tutorial on how to use Blockly  refer to [this video from the ER4STEM project](https://www.youtube.com/watch?v=mekb54UhSeA).

![Demo Management Page](./images/Workspace.png)

Blockly comes up with a lot of pre-defined blocks, which are here combined into seven categories (Logic, Loop, Math, Text, Lists, Variables, Functions). They are also include links to help pages for each block which can be accessed via *right click on the block -> Help*. Additionally to these blocks, there are blocks for handling dictionaries (category *Dictionaries*) and the following three categories which includes HOBBIT specific blocks:
- Move: blocks for moving the robot forward and backward, turn it and use its navigation
- Arm Control: any action related to the arm of the robot
- Interaction: providing connection to HOBBIT's tablet for user input and printing messages as well as controlling its eyes and head

Finally, there's the *Custom Blocks* category, where blocks are stored, which have been created using the tool's [block configurator](#create-custom-blocks).

# [Create custom blocks](#create-custom-blocks)
It is possible to create custom blocks for all three ROS communication patterns (topics, services and actions). The following sections provides examples on how to create custom blocks for each of them.

## Publish to topics

## Call a service

## Create an action client

# [UI Directory](#ui-directory)
This section provides an overview of the folder structure and important files of the tool, which could be useful when transfering it to another robot.

## /js
This folder includes all built libraries which are necessary to run the tool, e.g. the full built Blockly library, the code editor or supporting JavaScript and CSS frameworks, as well as code related to the tool itslef. The following files and folders are related to Blockly and therefore are relevant if a new version of Blockly should be used:
* blockly_compressed.js: all the core functionalities of Blockly
* blocks_compressed.js: appearence and specifications of pre-defined blocks
* python_compressed.js: Python generator functionalities including code definitions of pre-defined blocks

## /custom
This folder includes the manually modifications made to the pre-built Blockly library. If an version update of Blockly is desired it is necessary to include this modifications manually into the new version and re-build it. The files are listed in the following.

### /generators/python.js
Actually providing the generator functionality this file also includes functions which create the basic code skeleton (e.g. `if __name__=='__main__ ...'`) or advanced importing of packages.

### /generators/python
This folder includes two custom created files, which can be copied directly. The code which is generated for the pre-defined HOBBIT blocks is included in the `hobbit.js`. Since Blockly does not provide any functionality for Python dictionaries basic blocks for handling them are implemented inside `dict.js`.

### /blocks
This folder includes two files, `hobbit.js` and `dict.js`, which are related to the above mentioned blocks and provides the block specifications for it.

## /demos
Inside this folder the necessary files for saving demos are stored. The .xml files, which are interpreted by Blockly are stored inside the `xml` folder. They only include the blocks used and not any executable code. Therefore, the `src` folder include all the corresponding Python files, which are called by the interface. It is important that their names remain the same as in the `xml` directory.

## /HobbitLib.py
This is the Python module which provides all the ROS functionality, e.g. publishing to topics or calling services. It is automatically included in all generated files. If applicable it is necessary to copy it to the `/src` folder of the package where the tool is included. This is because of the fact, that the tool creates a temporary file (`run.py`) in this directory which is executed. To define another directory where this file is created please follow the steps described in the [first sections](#general).

## /toolbox.xml
This file describes how the toolbox of the [Blockly interface](#create-a-demo) is structured, e.g. which categories and blocks are provided and how they are clustered.

## /blocks.json
Inside this files all the specifications for custom blocks created with the provided [block configurator](#create-custom-blocks) are saved, e.g. design and code.