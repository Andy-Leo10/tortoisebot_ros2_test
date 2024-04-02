#!/bin/bash

# Use pgrep to find all PIDs associated with gzserver
pids=$(pgrep gzserver)

# Iterate over each PID and kill it
for pid in $pids
do
    kill $pid
done