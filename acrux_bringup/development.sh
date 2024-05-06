#!/bin/bash

# THIS SHELL SCRIPT IS WRITTEN BY RIGBETEL LABS LLP
# DO NOT MODIFY / CHANGE THIS SCRIPT AS IT CAN LEAD
# TO DEVESTATING DAMAGE TO THE ROBOT AND IT'S CRITI
# CAL FIRMWARE. PLEASE KEEP CAUTION WHILE EDITING .



systemctl --user disable robot_bringup.service
systemctl --user stop robot_bringup.service
loginctl disable-linger $USER
