#!/bin/bash
echo `ifconfig | grep wlp -A2 | grep inet | awk -F" " {'print $2'} | awk -F":" {'print $2'}`
