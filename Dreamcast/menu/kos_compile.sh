#!/bin/bash

##################################################################################################
# .sh script since Visual Studio Code does not seem to launch sequencial tasks in the same context,
#  hence loosing the environment variables... sigh...
#
# How to use: specify a task like this one in tasks.json, and pass any arguments you want:
#  {
#   "label": "DEBUG - build current file - KOS",
#   "type": "shell",            
#   "command": "${workspaceFolder}/kos_compile.sh",
#   "args": [
#            "-g",      //turn on debugging info
#            "-O0",     //make debugging produce the expected results (overrides KOS's -O2, since only the last option is effective)
#            "${file}", //the file to compile
#            "-o", "${workspaceFolder}/Debug/${fileBasenameNoExtension}.elf", //the output file
#            "-I", "${workspaceFolder}/abcd", //include directory
#            "-l", "GL", //link library
#           ],
#  }
##################################################################################################

#set the KOS environtment variables
source /opt/toolchains/dc/kos/environ.sh

#pass all parameters to kos-cc
kos-cc $@