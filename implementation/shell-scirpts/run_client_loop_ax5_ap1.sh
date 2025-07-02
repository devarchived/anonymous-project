#!/bin/sh

# ===== CONFIGURATION =====
STA_IP="192.168.3.2"                # IP of roaming STA (iperf3 server)
PORT="5202"                         # STA iperf3 server port
BITRATE="100M"                      # UDP bitrate
DURATION="60"                       # Test duration in seconds
RETRY_INTERVAL="5"                  # Time between retries
PACKET_SIZE="1474"
TEST_NUMBER="1"
SCENARIO_TYPE="AX5"                 # EHR, AX2, AX5, etc.
LOGFILE="/root/iperf3_client_ax5_ap1_${SCENARIO_TYPE}_${TEST_NUMBER}.log"

echo "Logging to $LOGFILE"

# ===== WAIT FOR STA TO BE REACHABLE =====
while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    echo "[$TIMESTAMP] Pinging STA $STA_IP..." | tee -a "$LOGFILE"
    
    ping -c 1 -W 1 "$STA_IP" >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "[$TIMESTAMP] STA is reachable. Proceeding with iperf3..." | tee -a "$LOGFILE"
        break
    fi
    
    echo "[$TIMESTAMP] STA not reachable. Retrying in ${RETRY_INTERVAL}s..." | tee -a "$LOGFILE"
    sleep "$RETRY_INTERVAL"
done

# ===== RUN IPERF3 TEST =====
while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    echo "[$TIMESTAMP] Trying iperf3 to $STA_IP:$PORT..." | tee -a "$LOGFILE"

    result=$(iperf3 -c "$STA_IP" -u -b "$BITRATE" -l "$PACKET_SIZE" -t "$DURATION" -p "$PORT" 2>&1)
    echo "$result" >> "$LOGFILE"

    if echo "$result" | grep -q "iperf3: error"; then
        echo "[$TIMESTAMP] iperf3 failed. Retrying in ${RETRY_INTERVAL}s..." | tee -a "$LOGFILE"
        sleep "$RETRY_INTERVAL"
    else
        echo "[$TIMESTAMP] iperf successful. Exiting..." | tee -a "$LOGFILE"
        break
    fi
done
