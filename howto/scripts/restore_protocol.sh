#!/bin/sh

DIR=/usr/lib/python2.7/dist-packages/autobahn/websocket
cd $DIR
sudo cp protocol.py.orig protocol.py
sudo rm protocol.py.orig

