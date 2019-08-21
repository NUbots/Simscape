# Simscape
Physics simulation using Simulink's [Simscape Multibody](https://www.mathworks.com/products/simmechanics.html)


## Acknowledgements
- Modelling work implemented within this codebase is partially based on the [Simscape simulation](https://github.com/mrl-hsl/Simulator) by MRL-HSL


## Requirements
1. [MATLAB](https://mathworks.com/products/matlab.html) (tested with R2018A v9.1)
2. [Simulink](https://mathworks.com/products/simulink.html)
3. [Simscape](https://mathworks.com/products/simscape.html) 
4. [Simscape Multibody](https://mathworks.com/products/simmechanics.html)


## Setup
- Navigate to the `src` folder in MATLAB's File Explorer
- To run the static simulation:
    - Run one of the script files, e.g. `zombie`
- To run the contact simulation:
    - Run `init` to establish various simulation parameters
    - Open up the contact simulation with `NUgus_contact` and press run (`CTRL+T`)