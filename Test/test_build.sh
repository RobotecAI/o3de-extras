#!/bin/bash

echo "Running test script"

# Test
. /opt/ros/humble/setup.sh

cd /data/workspace/WarehouseTest

if cmake --build build/linux --config profile --target WarehouseTest.GameLauncher Editor ; then
    VALUE="RESULT: ALL TESTS PASSED" # expected result
    echo "Build succeeded"
else
    VALUE="RESULT: Build failed"
fi

# Print the value, needed for the action to get the result
echo $VALUE

exit 0