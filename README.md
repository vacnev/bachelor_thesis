Structure:
- src/mdp.hpp provides an interface for RMDPs (cummulative probabilistic cMDPs) further extended for Hallway Game specific features
- src/history.hpp provides history data structure
- src/hallway contains Hallway Game scenarios used in experiments together with the Hallway Game source code in hallway.hpp following the interface from mdp.hpp

The source code of the algorithms is provided in src/debra and src/ralph, they both accept MDPs following the interface.
Setting the MDP used and its parameters is done in the main_X.cpp file. Settings the algorithms' hyperparametres is possible in the correspoding .hpp source files.

Compilation:

Download and unpack the OR-tools library from https://developers.google.com/optimization/install/cpp/binary_linux.
To compile the algorithm enter its folder, e.g., src/debra, and run the commands from src/cmake.txt:

1) cmake -S . -B build -DCMAKE_PREFIX_PATH="../../../or-tools_amd64_ubuntu-22.04_cpp_v9.6.2534/or-tools_x86_64_Ubuntu-22.04_cpp_v9.6.2534" , to DCMAKE_PREFIX_PATH enter your relative path to the unpacked OR-tools library
2) cmake --build build -v

Then enter the build folder (cd build) and run make (the folder and corresponding makefile were created by cmake).
The compiled binary will be available in the bin folder.

