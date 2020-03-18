#! /bin/bash

trap 'cleanup; echo >&2 "FAILED!"' 0
cleanup() { echo "Cleaning up"; kill $(jobs -p) >/dev/null 2>&1 || true ; sleep 1 ; }
clean_exit() {
    cleanup
    trap '' 0
    error_code=$1; shift
    echo >&2 "$@"
    exit $error_code
}

SHDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$( dirname $SHDIR )"
TOPIC="/chatter"
COUNT=10
HERTZ=1

export ROS_MASTER_URI="http://192.168.56.2:11311"

# load the setup script
source $DIR/devel/setup.bash

[ -z "$1" ] && echo "=== Local test without rvmaster"
[ -n "$1" ] && echo "=== Local test with rvmaster"

echo "Starting listener..."
rostopic echo "$TOPIC" &> "$DIR/listener.log" &

echo "Starting publisher..."
if [ -z "$1" ]; then
  python "$DIR/src/test/pubstringn.py" "rv/monitored$TOPIC" "A" "$COUNT" "$HERTZ" > "$DIR/publisher.log"
else
  python "$DIR/src/test/pubstringn.py"             "$TOPIC" "A" "$COUNT" "$HERTZ" > "$DIR/publisher.log"
fi

clean_exit
