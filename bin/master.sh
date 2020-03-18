#! /bin/bash

SHDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$( dirname $SHDIR )"

export ROS_MASTER_URI="http://localhost:11311"

if [ -z "$1" ]; then
  echo "=== Local test without rvmaster"
  echo "Starting rosmaster..."
  roscore         &> "$DIR/roscore.log"
else
  echo "=== Local test with rvmaster"
  echo "Starting rvmaster..."
  ACCESS_POLICY_PATH="$DIR/config/simple-access-policy.cfg" $DIR/bin/rvcore &> "$DIR/roscore.log"
fi
