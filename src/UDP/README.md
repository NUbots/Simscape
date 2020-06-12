# SimscapeUDP
Middleware between Simscape and robots

## Requirements
[C++ Protobuf](https://github.com/protocolbuffers/protobuf/tree/master/src)
Matlab with Simulink installed 

## Setting Up
1. Add mex to path or alias, `alias mex='/usr/local/MATLAB/R2020a/bin/mex'`.
2. Run `buildSimscapeUDP.sh` and make sure that the compiled files are in the matlab path
3. Open Matlab
4. Open MATLAB/SimscapeUDP.slx
5. Drag library blocks into your Simulink model
6. Add `platform::SimscapeUDP` to your role
7. Build your role

## Running
1. Make sure that your matlab workspace has the correct busses loaded, if not run `loadSimscapeUDP`
2. Run `./b run <your role>`
3. Run your Simulink model
