#!/bin/sh

# ===== CONFIGURATION =====
STA_IP="192.168.3.2"       		# Replace with STA IP (2.4 GHz or 5 GHz)
PORT="5202"                		# Match the STA's iperf3 server port
BITRATE="100M"            		# Target bitrate in bps
DURATION="60"               		# Duration per test (in seconds)
RETRY_INTERVAL="5"        		# Time to wait between retries (in seconds)
PACKET_SIZE="1474"
TEST_NUMBER="1"
SCENARIO_TYPE="EHR"			# EHR or AX2 or AX5
LOGFILE="/root/iperf3_client_ap2_${SCENARIO_TYPE}_${TEST_NUMBER}.log"  	# Log file (RAM-based in OpenWRT)

echo "Logging to $LOGFILE"

# ===== LOOP =====
while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    echo "[$TIMESTAMP] Trying iperf3 to $STA_IP:$PORT..." | tee -a "$LOGFILE"
    
    result=$(iperf3 -c "$STA_IP" -u -b "$BITRATE" -l "$PACKET_SIZE" -t "$DURATION" -p "$PORT" 2>&1)
    echo "$result" >> "$LOGFILE"

    if echo "$result" | grep -q "iperf3: error"; then
        echo "[$TIMESTAMP] STA not reachable, retrying in ${RETRY_INTERVAL}s..." | tee -a "$LOGFILE"
        sleep "$RETRY_INTERVAL"
    else
        echo "[$TIMESTAMP] iperf successful. Exiting..." | tee -a "$LOGFILE"
        break
    fi
done
