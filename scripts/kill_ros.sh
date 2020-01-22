#!/bin/bash
sudo kill -9 `ps -ef | grep ros | awk -F " " {'print $2'}`

