# Arexx Engineering SPOT

This repository is made for a SPOT prototype for Arexx Engineering.
Spot is a fun robot dog that uses an Micro:Bit V2.0 as it's brain.
The goal of this project is to make programming the robot dog fun and easy for children to do.

In this file I will explain the following things:
* [How to use this extension](README.md#how-to-use-this-extension)
* [How to edit this extension](README.md#how-to-edit-this-extension)
    * [Installing the necessarily programs](README.md#installing-the-necessarily-programs)
    * [Setting up the folder structure](README.md#setting-up-the-folder-structure)
    * [How to open and compile the project](README.md#how-to-open-and-compile-the-project)
* [Making custom blocks](README.md#making-custom-blocks)
    * [Controlling the movement of Spot](README.md#controlling-the-movement-of-spot)


## How to use this extension

To use this extension you need to create a new project or open an existing project.

1. Go to [Microsoft MakeCode for Micro:Bit](https://makecode.microbit.org/)
2. Sign in or create an account to be sure that your projects will be saved
3. Click on **New Project**
4. Give your project a name
5. Click on the gearwheel then click on **Extensions**
6. Search for **https://github.com/S1146468/Spot_Arexx_Engineering_PIB.git** and import it
7. You can now use this extension


## How to edit this extension

This project is built on the [pxt-microbit repository](https://github.com/microsoft/pxt-microbit.git).
In order to compile the project, you first need to setup an local development environment, which when set up correctly will allow you to edit and compile the project using a CLI.


### Installing the necessarily programs

To start setting up the environment you need to install the following programs in the following order:

1. [Node.js 8.9.4 or higher](https://nodejs.org/en)
2. [Yotta (follow manual install for Windows)](http://docs.yottabuild.org/#installing-on-windows)
3. [SRecord 1.64 and move it to C:\ ](https://sourceforge.net/projects/srecord/files/srecord-win32/1.64/)
4. [Visual Studio and/or the C++ toolchains](https://visualstudio.microsoft.com/downloads/)

Also, make sure you add these to your Path:
```
C:\Python27\Scripts
```
```
C:\srecord_dir
```


### Setting up the folder structure

1. Open the Node.js command prompt and make sure it is set on the following directory
```
C:/Users/YOUR_USERNAME
```

2. Clone the pxt repository
```
git clone https://github.com/microsoft/pxt
cd pxt
```

3. Install the dependencies of pxt and build it
```
npm install
npm run build
cd ..
```

4. Clone the pxt-common-packages repository
```
git clone https://github.com/microsoft/pxt-common-packages
cd pxt-common-packages
npm install
```

5. Link pxt-common-packages to pxt
```
npm link ../pxt
cd ..
```

6. Clone this repository
```
git clone https://github.com/microsoft/pxt-microbit
cd pxt-microbit
```

7. Install the PXT command line
```
npm install -g pxt
```

8. Install the pxt-microbit dependencies
```
npm install
```

9. Link pxt-microbit back to base pxt repository
```
npm link ../pxt
npm link ../pxt-common-packages
```

10. Check of the file structure looks as the follows
```
       makecode
          |
  ----------------------------------
  |       |                        |
 pxt      pxt-common-packages  pxt-microbit
```



### How to open and compile the project

1. Open the Node.js command prompt and make sure it is set on the following directory
```
C:/Users/YOUR_USERNAME
```

2. Open the projects folder
``` 
cd makecode/pxt-microbit/projects
```

3. Clone the Spot_Arexx_Engineering_PIB repository to the projects folder
```
git clone https://github.com/S1146468/Spot_Arexx_Engineering_PIB
cd Spot_Arexx_Engineering_PIB
```

4. Connect the library to the file structure
```
pxt install
```

5. Start coding in the project
```
code .
```

6. To compile the project 
``` 
pxt
```


## Making custom blocks

The useable blocks are defined in [Spot.ts](Spot.ts). 
For more information about defining your own blocks pleas visit [Defining blocks](https://makecode.com/defining-blocks).

The complete functionality of the movement of Spot is defined in [Spot.cpp](Spot.cpp) and is then linked via a shim to Static TypeScript.


### Controlling the movement of Spot

To be able to control the legs of Spot I made an inverse kinematic model that calculates all the servo angles of a given leg to make moving the leg easier.
The inverse kinematic model needs to know the X, Y, Z, Pitch, Roll and Yaw positions and the corresponding leg ID.

- X is for left to right
- Y is for forward and backward
- Z is for up and down
- Pitch rotates over the X-axis
- Roll rotates over the Y-axis
- Yaw rotates over the Z-axis

The code for moving all the legs to a certain position looks as follows:
```C++
SetDataForKinematics(float X, float Y, float Z, float Pitch, float Roll, float Yaw, int leg_ID);
SetDataForKinematics(float X, float Y, float Z, float Pitch, float Roll, float Yaw, int leg_ID);
SetDataForKinematics(float X, float Y, float Z, float Pitch, float Roll, float Yaw, int leg_ID);
SetDataForKinematics(float X, float Y, float Z, float Pitch, float Roll, float Yaw, int leg_ID);
DataProcessorKinematics(int speed);
```

For an example og how to use it go to [Spot.cpp](Spot.cpp#L254)

If you don't need to move a certain leg go to [Spot.cpp](Spot.cpp#L511) for an example.