#!/bin/sh

# ===== CONFIGURATION =====
STA_IP="192.168.2.2"       		# Replace with STA IP (2.4 GHz or 5 GHz)
PORT="5201"                		# Match the STA's iperf3 server port
BANDWIDTH="20M"            		# Bandwidth for UDP test
DURATION="10"               		# Duration per test (in seconds)
RETRY_INTERVAL="10"        		# Time to wait between retries (in seconds)
LOGFILE="/tmp/iperf3_client.log"  	# Optional log file (RAM-based in OpenWRT)

# ===== LOOP =====
while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    echo "[$TIMESTAMP] Trying iperf3 to $STA_IP:$PORT..." | tee -a "$LOGFILE"
    
    result=$(iperf3 -c "$STA_IP" -u -b "$BANDWIDTH" -t "$DURATION" -p "$PORT" 2>&1)
    echo "$result" >> "$LOGFILE"

    if echo "$result" | grep -q "iperf3: error"; then
        echo "[$TIMESTAMP] STA not reachable, retrying in ${RETRY_INTERVAL}s..." | tee -a "$LOGFILE"
        sleep "$RETRY_INTERVAL"
    else
        echo "[$TIMESTAMP] iperf successful. Continuing..." | tee -a "$LOGFILE"
        # Optional: break after first success if needed
        # break
        sleep "$RETRY_INTERVAL"
    fi
done
