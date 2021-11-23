#!/bin/sh
JC=`cat java_cmd_string`
xterm -e /bin/bash -l -c "$JC test.middleware.ROSLauncher $1 gui > /dev/null"
