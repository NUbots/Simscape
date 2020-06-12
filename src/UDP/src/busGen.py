import glob
import os
import google.protobuf.descriptor_pb2

def first_lower(s):
   if len(s) == 0:
      return s
   else:
      return s[0].lower() + s[1:]

def getType(field):
    return {
        11:lambda: field.type_name.split(".")[-1],#MESSAGE
        14:lambda: field.type_name.split(".")[-1],#ENUM
        1:lambda:"double",
        2:lambda:"float",
        5:lambda:"int32_t",
        3:lambda:"int32_t",#64
        13:lambda:"uint32_t",
        4:lambda:"uint32_t",#64
        17:lambda:"int32_t",
        18:lambda:"int32_t",#64
        7:lambda:"uint32_t",
        6:lambda:"uint32_t",#64
        15:lambda:"int32_t",
        16:lambda:"int32_t",#64
        9:lambda:"string",#String
        8:lambda:"bool",
        12:lambda:"string",#Bytes
    }[field.type]()

def getFullType(field):
    return {
        11:lambda: field.type_name.replace(".","::"),#MESSAGE
        14:lambda: field.type_name.replace(".","::"),#ENUM
        1:lambda:"double",
        2:lambda:"float",
        5:lambda:"int32_t",
        3:lambda:"int32_t",#64
        13:lambda:"uint32_t",
        4:lambda:"uint32_t",#64
        17:lambda:"int32_t",
        18:lambda:"int32_t",#64
        7:lambda:"uint32_t",
        6:lambda:"uint32_t",#64
        15:lambda:"int32_t",
        16:lambda:"int32_t",#64
        9:lambda:"string",#String
        8:lambda:"bool",
        12:lambda:"string",#Bytes
    }[field.type]()

def isRepeated(field):
    return {
        1:lambda:False,
        2:lambda:False,
        3:lambda:True,#Repeated field
    }[field.label]()

def makeBus(message):
    out = ""
    for subMessage in message.nested_type:
        out += makeBus(subMessage)
    
    out += "typedef struct {\n"
    if(message.field):
        out += "    alignas(8)"
    for field in message.field:
        if(isRepeated(field)):
            if(field.type == 11 or field.type == 14):#messag or enum
                out += "    {1}Bus {0}[20];\n".format(field.name,getType(field))
            else:
                out += "    {1} {0}[20];\n".format(field.name,getType(field))#TODO(cameron) Find a better way
        else:
            if(field.type == 11 or field.type == 14):#messag or enum
                out += "    {1}Bus {0};\n".format(field.name,getType(field))
            else:
                out += "    {1} {0};\n".format(field.name,getType(field))

    out += "}} {}Bus;\n\n".format(message.name)
    return out

def makeToMessage(message,nested_name_specifier):
    out = ""
    for subMessage in message.nested_type:
            out += makeToMessage(subMessage,"{}::".format(message.name))

    out += "{2}{0} {1}BusCopy(const {0}Bus& in){{\n".format(message.name,first_lower(message.name),nested_name_specifier)
    out += "    {1}{0} returnable;\n".format(message.name,nested_name_specifier)
    for field in message.field:
        if(isRepeated(field)):
            out += "    for(size_t i = 0; i < sizeof(in.{0})/sizeof(in.{0}[0]); i++ ){{\n".format(field.name)
            out += "        returnable.add_{0}()->operator=({2}BusCopy(in.{0}[i]));\n".format(field.name,getFullType(field),first_lower(getType(field)))
            out += "    }\n"
        else:
            if(field.type == 11):#type is another message
                out += "    returnable.set_allocated_{1}(new {0}({2}BusCopy(in.{3})));\n".format(getFullType(field),field.name.lower(),first_lower(getType(field)),field.name,nested_name_specifier)
            else:
                out += "    returnable.set_{0}(in.{1});\n".format(field.name.lower(),field.name)
    out += "    return returnable;\n"
    out += "}\n\n"
    return out

def makeFromMessage(message,nested_name_specifier):
    out = ""
    for subMessage in message.nested_type:
            out += makeFromMessage(subMessage,"{}::".format(message.name))
    out += "{0}Bus {1}BusCopy(const {2}{0}& in){{\n".format(message.name,first_lower(message.name),nested_name_specifier)
    out += "    {0}Bus returnable;\n".format(message.name,nested_name_specifier)
    for field in message.field:
        if(isRepeated(field)):
            out += "    for(size_t i = 0; i < in.{0}_size(); i++ ){{\n".format(field.name)
            out += "        returnable.{0}[i] = {2}BusCopy(in.{0}(i));\n".format(field.name,getFullType(field),first_lower(getType(field)))
            out += "    }\n"
        else:
            if(field.type == 11):#type is another message
                out += "    returnable.{3} = {2}BusCopy(in.{1}());\n".format(getFullType(field),field.name.lower(),first_lower(getType(field)),field.name,nested_name_specifier)
            else:
                out += "    returnable.{1} = in.{0}();\n".format(field.name.lower(),field.name)
    out += "    return returnable;\n"
    out += "}\n\n"
    return out

protoDescription = google.protobuf.descriptor_pb2.FileDescriptorSet()

f = open("message.descriptor","rb")
protoDescription.ParseFromString(f.read())
f.close()

for file in protoDescription.file:
    hFile = open("{}Bus.h".format(file.name.split("/")[-1].split(".")[0]),"w")

    hFile.write(
"""#ifndef {0}Bus_CameronMurtagh
#define {0}Bus_CameronMurtagh

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include <stdint.h>
#include <stdalign.h>

""".format(file.name.split("/")[-1].split(".")[0])
    )

    for dependancy in file.dependency:
        hFile.write("#include \"{}Bus.h\"\n\n".format(dependancy.split("/")[-1].split(".")[0]))

    for message in file.message_type:
        hFile.write(makeBus(message))

    hFile.write("#endif\n")

    hFile.close()

    methodshFile = open("{}BusMethods.h".format(file.name.split("/")[-1].split(".")[0]),"w")
    methodshFile.write(
"""#ifndef {0}BusMethods_CameronMurtagh
#define {0}BusMethods_CameronMurtagh

#include "{1}.pb.h"
#include "{0}Bus.h"

#include <vector>
""".format(file.name.split("/")[-1].split(".")[0],file.name.split(".")[0]))

    for dependancy in file.dependency:
        methodshFile.write("#include \"{0}Bus.h\"\n#include \"{0}BusMethods.h\"\n".format(dependancy.split("/")[-1].split(".")[0]))
    
    methodshFile.write("\n")

    for message in file.message_type:
        for subMessage in message.nested_type:
            methodshFile.write("{1}::{3}::{0} {2}BusCopy(const {0}Bus& in);\n".format(subMessage.name,file.package.replace(".","::"),first_lower(subMessage.name),message.name))
        methodshFile.write("{1}::{0} {2}BusCopy(const {0}Bus& in);\n".format(message.name,file.package.replace(".","::"),first_lower(message.name)))
        for subMessage in message.nested_type:
            methodshFile.write("{0}Bus {2}BusCopy(const {1}::{3}::{0}& in);\n".format(subMessage.name,file.package.replace(".","::"),first_lower(subMessage.name),message.name))
        methodshFile.write("{0}Bus {2}BusCopy(const {1}::{0}& in);\n".format(message.name,file.package.replace(".","::"),first_lower(message.name)))
    methodshFile.write("\n#endif\n")

    methodshFile.close()

    methodscppFile = open("{}BusMethods.cpp".format(file.name.split("/")[-1].split(".")[0]),"w")
    methodscppFile.write("#include \"{}BusMethods.h\"\n#include <google/protobuf/util/time_util.h>\n\n".format(file.name.split("/")[-1].split(".")[0]))
    methodscppFile.write("using namespace {0};\n".format(file.package.replace(".","::"),file.name.split("/")[-1].split(".")[0]))
    
    for message in file.message_type:
        methodscppFile.write(makeToMessage(message,"{}::".format(file.package.replace(".","::"))))
        methodscppFile.write(makeFromMessage(message,"{}::".format(file.package.replace(".","::"))))

    methodscppFile.close()