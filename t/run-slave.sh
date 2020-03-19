#! /bin/bash

if [ "$#" -ne 1 ] && [ "$#" -ne 2 ]; then
  echo "usage: run-slave.sh <IP-A> [on]"
  exit 1
fi

trap 'cleanup; echo >&2 "FAILED!"' 0
cleanup() { echo "Cleaning up"; kill $(jobs -p) >/dev/null 2>&1 || true ; sleep 1 ; }
clean_exit() {
    cleanup
    trap '' 0
    error_code=$1; shift
    echo >&2 "$@"
    exit $error_code
}

IP_A=$1
ON_STATE=$2

SHDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$( dirname $SHDIR )"
TOPIC="/chatter"
COUNT=1000
HERTZ=30

export ROS_MASTER_URI="http://$IP_A:11311"

# load the setup script
source $DIR/devel/setup.bash

[ -z "$ON_STATE" ] && echo "=== Local test without rvmaster"
[ -n "$ON_STATE" ] && echo "=== Local test with rvmaster or unmonitored"

echo "Starting subscriber..."
rostopic echo "$TOPIC" &> "$DIR/subscriber.log" &

echo "Starting publisher..."
if [ -z "$ON_STATE" ]; then
  python "$DIR/src/test/pubstringn.py" "rv/monitored$TOPIC" "A" "$COUNT" "$HERTZ" > "$DIR/publisher.log"
else
  python "$DIR/src/test/pubstringn.py"             "$TOPIC" "A" "$COUNT" "$HERTZ" > "$DIR/publisher.log"
fi

clean_exit
