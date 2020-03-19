#! /bin/bash

if [ "$#" -ne 1 ] && [ "$#" -ne 2 ]; then
  echo "usage: run-monitor.sh <IP-A> [on]"
  exit 1
fi

SHDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$( dirname $SHDIR )"

IP_A=$1
ON_STATE=$2

export ROS_MASTER_URI="http://$IP_A:11311"

# load the setup script
source $DIR/devel/setup.bash

if [ -z "$ON_STATE" ]; then
  echo "=== Local test without rvmaster"
  echo "Starting monitor..."
  rosrun rvmonitor monitor-fixed-output  &> "$DIR/monitor.log"
else
  echo "=== Local test with rvmaster"
  echo "Starting monitor..."
  rosrun rvmonitor monitor-fixed-output --with-rvmaster &> "$DIR/monitor.log"
fi
