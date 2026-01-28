# simulation Directory

These tools are meant to act as drop-in replacements for the real sensor drivers. 

`Serial_Sim.h` is the central piece. It collects simulated data from the serial port and updates its internal data. 
The simulated sensors (e.g. `Serial_Sim_BMP390.h`, ...) then pull data from `Serial_Sim.h` instead of the real sensors.

In your main code of the flight computer you can simply include these sensor drivers instead of the real ones when you want to run a simulation. You'll have to run `simulation.py` on a computer connected to the flight computer's serial port to send the simulated data once the flight computer starts running. 