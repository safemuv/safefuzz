#!/bin/sh

DIR=/usr/lib/python2.7/dist-packages/autobahn/websocket
sudo cp ../patches/python_protocol_patch.py $DIR
cd $DIR
sudo cp protocol.py protocol.py.orig
sudo patch < python_protocol_patch.py

