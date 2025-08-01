# Orbital Sim
This is a simple (and slightly slow) satellite orbital simulation software I am working on to help aid some of my work in Husky Satellite Lab on the ADC susbsystem.
## Current Features
The simulation models translational and rotational dynamics using an RK4 integrator. There is magnetorquer model set to detumble the satellite, which utilizes a model for Earth's magnetic field.
## Features coming soon
The next thing I have coming up is sensors and noise, and then adding in reaction wheels too.
## How to use/run
All of the satellite parameters are in `params.py`. Here, you can change almost everything to do with the satellite, except for some of the orbital parameters (initial position). Changing the parameters in `main.py` would change your orbit shape (from circular to elliptical), if you would like. The main other thing you might want to mess with is the magnetorquer controller gain in `satellite.py`.
## Acknowledgements
This simulation is partially adapted/based off of [Dr. Carlos Montalvo](https://github.com/cmontalvo251) ADCS seminar series. I also used his [book](https://github.com/cmontalvo251/LaTeX/blob/master/Aerospace_Mechanics/aerospace_mechanics.pdf) as a refrence throughout, especially on some of the equations and conversions I wasn't too familar with.