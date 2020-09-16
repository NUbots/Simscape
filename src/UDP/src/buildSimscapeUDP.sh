#!/bin/sh

alias mex='/usr/local/MATLAB/R2020a/bin/mex'

protoc --include_imports --proto_path=../src/proto/ --descriptor_set_out=message.descriptor ../src/proto/*.proto
protoc --cpp_out=./ /usr/include/google/protobuf/timestamp.proto
protoc --proto_path=../src/proto/ --cpp_out=. ../src/proto/*.proto

python3 ../src/busGen.py

mex -g -I/usr/local/include -I../build -L/usr/local/lib/ -lprotobuf ../src/SfunProtobufEncode.cpp DarwinSensors.pb.cc DarwinSensorsBusMethods.cpp timestampBusMethods.cpp
mex -g -I/usr/local/include -I../build -L/usr/local/lib/ -lprotobuf ../src/SfunProtobufDecode.cpp ServoTarget.pb.cc ServoTargetBusMethods.cpp timestampBusMethods.cpp
mex -g -I/usr/local/include -I../build -L/usr/local/lib/ -lprotobuf ../src/SfunProtobufServosEncode.cpp DarwinSensors.pb.cc DarwinSensorsBusMethods.cpp timestampBusMethods.cpp