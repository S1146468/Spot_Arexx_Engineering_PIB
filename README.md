# Arexx Engineering SPOT

This repository is made for a SPOT prototype for Arexx Engineering.
Spot is a fun robot dog that uses an Micro:Bit V2.0 as it's brain.
The goal of this project is to make programming the robot dog fun and easy for children to do.

In this file I will explain the following things:
* [How to use this extention](README.md#how-to-use-this-extention)
* [How to edit this extention](README.md#how-to-edit-this-extention)
    * [Installing the nessisairly programs](README.md#installing-the-nessisairly-programs)
    * [Setting up the folder structure](README.md#setting-up-the-folder-structure)
    * [How to open and compile the project](README.md#how-to-open-and-compile-the-project)
* [Making custom blocks](README.md#making-custom-blocks)


## How to use this extention

To use this extention you need to create a new project or open an exsisting project.

1. Go to [Microsoft MakeCode for Micro:Bit](https://makecode.microbit.org/)
2. Sign in or create an account to be sure that your projects will be saved
3. Click on **New Project**
4. Give your project a name
5. Click on the gearwheel then click on **Extentions**
6. Search for **https://github.com/S1146468/Spot_Arexx_Engineering_Github_2.git** and import it
7. You can now use this extention


## How to edit this extention

This project is built on the [pxt-microbit repository](https://github.com/microsoft/pxt-microbit.git).
In order to compile the project, you first need to setup an local developement envirement, which when set up correctly will allow you to edit and compile the project using a CLI.


### Installing the nessisairly programs

To start setting up the enviroument you need to install the following programs in the following order:

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

3. Clone the Spot_Arexx_Engineering_Github_2 repository to the projects folder
```
git clone https://github.com/S1146468/Spot_Arexx_Engineering_Github_2
cd Spot_Arexx_Engineering_Github_2
```

4. Connect the library to the file structure
```
pxt target microbit
pxt install
```

5. Start coding in the project
```
code .
```

6. To complile the project 
``` 
pxt
```


## Making custom blocks

The useable blocks are defined in [Spot.ts](Spot.ts)