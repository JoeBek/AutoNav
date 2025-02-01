This file is a guide for how to set up, run, and use the manual control system.

!!! WARNING see usage before running. This script has some speed differences from the motor controller. 
Use care when running and always have someone manning the physical e-stop.

# Install instructions

the requirements for running this are:
python 3.10+ (Earlier versions might work)

You can check your python version by opening the command prompt and typing `python3 --version`
go look up a tutorial on how to upgrade or install python for at least 3.10 if things don't work.


# setup instructions

once you have a working version of python, open a command prompt and navigate to the `tempcontrol` directory.
(Look in the `Scripts` directory from the top level of the repository)
this can be done by using `ls` to list directories and `cd` to go into a directory. 

Once in the `tempcontrol` directory, run the following command:

`python3 -m venv .venv` 

This creates a virtual environment to hold and isolate depencies

---

Then, if you are on Linux/Mac:

`source .venv/bin/activate`

or, if you are on windows:

`.venv\Scripts\activate`

This activates the virtual environment you just created.
You can leave the virtual environment at any point by running
`deactivate`

---

Finally, install the requirements by running the following:

`pip install -r requirements.txt`

This should install all the dependencies into the virtual environment.

---

# how to run / usage

to run the script, first activate the environment (see above) (not nessecary if it is already active)

then run `python3 drivebowser.py PORT`

where PORT should be substituted for the name of the serial port you are connected to the motor controller with.

This will launch the application. There is a guide within the application that appears in the terminal where you launched it from that explains application usage.

## speed information (important)

There is a 'step size' parameter that controls the resolution for robot speed. This parameter is set by default to 10, but may in the future be changed in the program through the menu options (not adding this until asked to). The step size will act a divider for the default motor speed resolution, which is 1000. 

This means that the 
MAX SPEED FOR THE ROBOT IS 100 FOR THIS PROGRAM OUT OF THE BOX.
when using the speed up and speed down commands, the speed is shown in relative terms to the step size.

The reason this is done is to save some life on your poor 'e' and 'q' keys, and to somewhat simply the choice for wheel speeds. If finer resolution is necessary, please annoy a software person about it or lock in and figure out how to change it in the code.







