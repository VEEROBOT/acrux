#!/bin/bash

# THIS SHELL SCRIPT IS WRITTEN BY RIGBETEL LABS LLP
# DO NOT MODIFY / CHANGE THIS SCRIPT AS IT CAN LEAD
# TO DEVESTATING DAMAGE TO THE ROBOT AND IT'S CRITI
# CAL FIRMWARE. PLEASE KEEP CAUTION WHILE EDITING .

sudo cp -f services/robot_bringup.service /etc/systemd/user/

systemctl --user enable robot_bringup.service
systemctl --user start robot_bringup.service

loginctl enable-linger $USER