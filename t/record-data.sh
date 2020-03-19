#!/bin/bash

if [ "$#" -ne 3 ]; then
  echo "usage: record-data.sh <test-name> <net-interface> <IP-B>"
  exit 1
fi

SHDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$( dirname $SHDIR )"

TEST_NAME=$1
NET_IFACE=$2
IP_B=$3
DATA_FILE="$TEST_NAME.pcap"
RESULT_FILE="$TEST_NAME-results.log"

echo "Recording data..."
sudo tshark -i "$NET_IFACE" -f "tcp and host $IP_B" -w "/tmp/$DATA_FILE"
echo "Saving data..."
sudo chmod a+rw "/tmp/$DATA_FILE"
cp "/tmp/$DATA_FILE" "$DIR"
echo "Recording results..."
tshark -r "$DIR/$DATA_FILE" -z io,stat,0,"SUM(frame.len)frame.len" > "$DIR/$RESULT_FILE"
echo "Done"
