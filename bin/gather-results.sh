#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "usage: gather-results.sh <test-name>"
fi

SHDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$( dirname $SHDIR )"

TEST_NAME=$1
DATA_FILE="$TEST_NAME.pcap"
RESULT_FILE="$TEST_NAME-results.txt"

echo "Recording data..."
sudo tshark -i enp0s8 -f "tcp and host 192.168.56.3" -w "/tmp/$DATA_FILE"
echo "Saving data..."
sudo chown ubuntu "/tmp/$DATA_FILE"
cp "/tmp/$DATA_FILE" "$DIR"
echo "Recording results..."
tshark -r test.pcap -z io,stat,0,"SUM(frame.len)frame.len" > "$DIR/$RESULT_FILE"
echo "Done"
