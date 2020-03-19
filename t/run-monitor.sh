#! /bin/bash

SHDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$( dirname $SHDIR )"

export ROS_MASTER_URI="http://192.168.56.2:11311"

# load the setup script
source $DIR/devel/setup.bash

if [ -z "$1" ]; then
  echo "=== Local test without rvmaster"
  echo "Starting monitor..."
  rosrun rvmonitor monitor-fixed-output  &> "$DIR/monitor.log"
else
  echo "=== Local test with rvmaster"
  echo "Starting monitor..."
  rosrun rvmonitor monitor-fixed-output --with-rvmaster &> "$DIR/monitor.log"
fi
