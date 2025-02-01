This file is a guide for how to set up, run, and use the manual control system.

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



