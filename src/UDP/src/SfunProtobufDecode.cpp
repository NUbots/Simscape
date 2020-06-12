#define S_FUNCTION_NAME  SfunProtobufDecode
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include "ServoTarget.pb.h"
#include <google/protobuf/util/time_util.h>
#include "ServoTargetBus.h"
#include "ServoTargetBusMethods.h"
#include <string>
#include <iostream>

/*	
 * Helper function that returns true if time1 is after time2	
 */
static bool timeTest(const google::protobuf::Timestamp &time1, const google::protobuf::Timestamp &time2)	
{
    if(time1.seconds() > time2.seconds())
    {
        return true;
    }
    else if(time1.seconds() == time2.seconds() && time1.nanos() > time2.nanos())
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * Initialise Ports
 */
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);

    /* manually check if the paramater count was set properly */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return; //Parameter mismatch reported by the Simulink engine
    }

    //Set the number of input ports to 1 and check that this was done successfully
    if (!ssSetNumInputPorts(S, 1)) 
    {
        return;
    }
    //Set the dimensions of the inport port to a dynamic size
    ssSetInputPortMatrixDimensions(S,0,DYNAMICALLY_SIZED,1);
    //Set that the inport will be used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    //Set the input port to a uint8
    ssSetInputPortDataType(S, 0, SS_UINT8);
    //Set the input port dimensions mode to be inherited
    ssSetInputPortDimensionsMode(S, 0, INHERIT_DIMS_MODE);
    //Specifies that the signal elements entering the specified port must occupy contiguous areas of memory.
    ssSetInputPortRequiredContiguous(S,0,true);

    //Set the number of output ports to 1 and check that this was done successfully
    if (!ssSetNumOutputPorts(S,1)) 
    {
        return;
    }
    //Set the width of the output port to be equal 20
    ssSetOutputPortWidth(S, 0, 20);
    
    //We are only using 1 sample time
    ssSetNumSampleTimes(S, 1);

    //set the number of work pointers to 1
    ssSetNumPWork(S, 1);
}

/*
 * Initialize time steps
 */
#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
    //Use the same sample time as incoming signal
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    //Don't offest the sample time
    ssSetOffsetTime(S, 0, 0.0);
}

/*
 * Set the width of the input vector
 */
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    // Set to the suggested width (e.g. the output width
    // from the connected block)
    ssSetInputPortDimensionInfo(S, port, dimsInfo);
}

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port,const DimsInfo_T *dimsInfo)
{
    //Don't change the output dimensions
    UNUSED_ARG(S);
    UNUSED_ARG(port);
    UNUSED_ARG(dimsInfo);
}    

/*
 * Initialisation of work vectors
 */
#define MDL_START
void mdlStart(SimStruct *S)
{
    //Create a array to store the values of the servos
    message::motion::ServoTarget *targetArray = new message::motion::ServoTarget[ssGetOutputPortWidth(S,0)];
    
    for(int i = 0; i < ssGetOutputPortWidth(S,0); i++)
    {
        //Initialise each element in the array
        //Create a TimeStamp object on the heap the give it to the protobuf object
        //who will now have ownership of it and will deallocate it for me
        //The assignment operator of ServoTargets does deallocate
        google::protobuf::Timestamp *epoch = new google::protobuf::Timestamp(google::protobuf::util::TimeUtil::GetCurrentTime());
        targetArray[i].set_allocated_time(epoch);
        targetArray[i].set_id(i);
        targetArray[i].set_position(0);
        targetArray[i].set_gain(0);
        targetArray[i].set_torque(0);
    }
    
    //Set the value of the 0th work pointer to the pointer to the array of ServoTarget
    ssSetPWorkValue(S, 0, targetArray);
}

/*
 * Calculate the output
 */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
    //Get a pointer to the work vector
    message::motion::ServoTarget *targetArray = (message::motion::ServoTarget *)ssGetPWorkValue(S, 0);

    //Get a pointer to the input vector
    uint8_T * input = (uint8_T *)ssGetInputPortSignal(S, 0);
    //Get the size of the input vector
    int_T iwidth = ssGetCurrentInputPortDimensions(S, 0, 0);
    
    //Convert pointer to array into string
    std::string buffer = std::string(input,input + iwidth);
    
    //A protobuf object for storing the incoming message
    message::motion::ServoTargets inMessage;
    //Parse the incoming binary data to the probobuf object
    inMessage.ParseFromString(buffer);
    for(int i = 0; i < inMessage.targets_size(); i++)
    {
        //Check that the id is within limits and check that the message is newer
        if(inMessage.targets(i).id() < ssGetOutputPortWidth(S,0) /*&&
           timeTest(inMessage.targets(i).time(),targetArray[inMessage.targets(i).id()].time())*/)
        {
            //Copy the new ServoTarget into the array
            targetArray[inMessage.targets(i).id()] = inMessage.targets(i);
        }
    }
    
    //Get a pointer to the output array
    real_T *output = ssGetOutputPortRealSignal(S,0);
    
    //Output all the positions
    for(int i = 0; i < ssGetOutputPortWidth(S,0); i++)
    {
        output[i] = targetArray[i].position();
    }
    
    UNUSED_ARG(tid);
}

/*
 * Clean up
 */
static void mdlTerminate(SimStruct *S)
{
    delete[] (message::motion::ServoTarget *)ssGetPWorkValue(S, 0);
}

#ifdef MATLAB_MEX_FILE    /* Is this file being compiled as a 
                             MEX-file? */
#include "simulink.c"     /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"      /* Code generation registration 
                             function */
#endif