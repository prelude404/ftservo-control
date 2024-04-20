#!/bin/bash
gnome-terminal --geometry 50x10+1050+10 -- bash config.sh & sleep 5
gnome-terminal --geometry 50x10+1050+10 -- bash Servo12Control.sh & sleep 1
gnome-terminal --geometry 50x10+1050+230 -- bash Servo34Control.sh & sleep 1
gnome-terminal --geometry 50x10+1050+450 -- bash Servo56Control.sh & sleep 1
gnome-terminal --geometry 50x10+1050+670 -- bash Servo78Control.sh & sleep 1

