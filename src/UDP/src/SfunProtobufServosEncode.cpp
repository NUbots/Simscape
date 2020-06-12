#define S_FUNCTION_NAME  SfunProtobufServosEncode
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include "DarwinSensors.pb.h"
#include "DarwinSensorsBus.h"
#include "DarwinSensorsBusMethods.h"
#include <string>

/*
 * Initialise Ports
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);

    /* manually check if the paramater count was set properly */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return; //Parameter mismatch reported by the Simulink engine
    }
    
    //Set custom data type
    int dType;
    char const* name = "ServosBus";
    ssRegisterTypeFromNamedObject(S, name, &dType);

    //In Ports
    //Sets number of inports to 1 and returns if failed
    if (!ssSetNumInputPorts(S, 1)) 
    {
        return;
    }
    //Set size of 0th array to 1
    ssSetInputPortWidth(S,0,1);

    //This says that we will use it in the model outputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    //Set the input to be the bus
    ssSetInputPortDataType(S, 0, dType);
    
    //Tell simulink that the input is a nonvirtual bus
    ssSetBusInputAsStruct(S, 0 , true);

    //Out Ports
    if (!ssSetNumOutputPorts(S,1)) 
    {
        return;
    }
    //Set the width of the output vector to 8192
    ssSetOutputPortWidth(S, 0, 2048);
    //Set the data type of the output to unsigned int 8
    ssSetOutputPortDataType(S, 0, SS_UINT8);
    
    //Set number of sample times that the block has
    ssSetNumSampleTimes(S, 1);
}

/*
 * Initialize time steps
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    //Set the 0th time state to be the one inherited by the income port
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    //Set the time offset to 0
    ssSetOffsetTime(S, 0, 0.0);
}

/*
 * Calculate the output
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    //Array of each inport, each inport is an array.
    InputPtrsType inputs = ssGetInputPortSignalPtrs(S,0);
    
    //Cast input to the struct
    ServosBus inBus = ((ServosBus **)inputs)[0][0];
    
    //Copy over the stuff
    message::platform::darwin::DarwinSensors::Servos protobufobj = servosBusCopy(inBus);
    
    //Make a string object to point at the serialized binary protobuf message
    std::string buffer = std::string();
    protobufobj.SerializeToString(&buffer);
    
    //Get a pointer to the start of the output array
    uint8_T *output = (uint8_T*)ssGetOutputPortSignal(S,0);

    //Try not to just segfault
    if(buffer.length() > ssGetOutputPortWidth(S,0))
        return;
    
    //Copy the string into the output array
    for (int i = 0; i < buffer.length(); i++) {
        output[i] = buffer[i];
    }
    UNUSED_ARG(tid);
}

/*
 * Clean up
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

#ifdef MATLAB_MEX_FILE    /* Is this file being compiled as a 
                             MEX-file? */
#include "simulink.c"     /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"      /* Code generation registration 
                             function */
#endif