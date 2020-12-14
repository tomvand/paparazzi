#!/bin/bash
# Reroute serial data to support multiple downlink channels and a separate
# uplink channel.
#
# Requires socat:  sudo apt install socat

# Input parameters
DOWNLINK1=${DOWNLINK1:-/dev/ttyACM0}
DOWNLINK2=${DOWNLINK2:-/dev/ttyACM1}
UPLINK=${UPLINK:-/dev/ttyACM1}

# Merge downlink streams
socat -d -d -u -b2048 gopen:$DOWNLINK1,rawer pipe:/tmp/downlink_pipe &
socat -d -d -u -b2048 gopen:$DOWNLINK2,rawer pipe:/tmp/downlink_pipe &

# Create pty for uplink/downlink streams
exec socat -d -d -x pipe:/tmp/downlink_pipe\!\!gopen:$UPLINK pty,link=/tmp/ttyMUX0
