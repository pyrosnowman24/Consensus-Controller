#!/bin/bash
sudo kill -9 `ps -ef | grep gzserver | grep -v grep | awk -F" " {'print $2'}`
