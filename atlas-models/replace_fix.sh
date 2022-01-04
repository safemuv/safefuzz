#!/bin/sh

if [ $# -eq 0 ]
  then
    echo "Usage: ./replace_fix.sh <username> to replace the username in the model file"
else
	OUTPUT_USER=$1;
	find . -name "*.model" -type f | xargs sed -E -i.bak "s/home\/([[:alnum:]]+)/home\/$OUTPUT_USER/g";
fi
