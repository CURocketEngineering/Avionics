# include Directory

This directory contains all the header files for the Avionics library. Each header file declares classes, functions, and constants that are used throughout the library. In essence, you can treat each header file as a discrete module/tool that can be utilized in your flight computer software. 

## Files
- `CommandNames.h`: Defines integer constants to represent various commands that can be sent to the flight computer. 
- `PowerManagement.h`: Declares a tool for reading ADC pins to monitor battery voltage. 
- `UARTCommandHandler.h`: Declares a tool for creating a shell-like interface over UART to send commands and receive status messages from the flight computer. The flight computer can declare and register commands with this interface. 


## Subdirectories

### data_handling

The data_handling subdirectory contains all tools pertaining to data management. It includes tools for logging data to various storage mediums as well as sending data for a radio. 

### simulation

The simulation subdirectory contains tools for simulating sensors in order to perform full code tests without an actual launch. It offers simulated drivers for various sensors that can drop-in replace the real sensor drivers. Overtime, the number of simulated sensors will grow to cover all sensors utilized by our club's flight computers. 

### state_estimation

The state_estimation subdirectory contains tools for estimating the rocket's state. The client simply initalizes these tools and then calls `update` functions with new data. In return, the tools provide current estimate states of the rocket or stage of flight. 