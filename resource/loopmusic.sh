#!/bin/bash

cleanup() {
    killall aplay
    exit 0
}

MUSIC_DIR=$1

trap cleanup SIGTERM

while true; do
    aplay -q $MUSIC_DIR & APID=$!; wait $APID; sleep 1
done
