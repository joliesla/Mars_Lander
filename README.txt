This program simulates the orbital mechanics of a Mars lander and plots its trajectory. Parameters such as initial position and 
velocity can be changed my adjusting the vel and pos variables in world.cpp (line 110 and 107 respectively). The mass of the planet along with other parameters is 
also variable.

The C++ file was created with notepad since the computer used to build this program had no administrator rights and hence,
an IDE was not available. Running the .exe will perform the integration and input the data into data2.txt which is extracted
and plotted in orb.xlsx. If training is turned on, data will be inserted into improve.txt and extracted by improvement.xlsx
to find the ideal values of the control parameters. The program is created to work with a second planet.

I personally use world.bat to run my simulations but if you have a C++ IDE, opening the .cpp file and running it should do the job.
