#!/bin/sh

# ===== CONFIGURATION =====
SERVER_IP="192.168.3.2"
PORT="5202"
TEST_NUMBER="5"
SCENARIO_TYPE="EHR"
LOGFILE="/root/iperf3_server_ap2_${SCENARIO_TYPE}_${TEST_NUMBER}.log"
RETRY_INTERVAL="5"
INTERFACE="phy1-sta0"
PING_TARGET="192.168.3.1"

echo "Logging to $LOGFILE"

# ===== FUNCTION =====
wait_for_ip_and_ping() {
    while true; do
        ip addr show "$INTERFACE" | grep -q "$SERVER_IP"
        if [ $? -eq 0 ]; then
            ping -c 1 -W 1 "$PING_TARGET" >/dev/null 2>&1
            if [ $? -eq 0 ]; then
                return 0
            fi
        fi
        echo "[$(date +"%Y-%m-%d %H:%M:%S")] Waiting for IP and ping to $PING_TARGET..." | tee -a "$LOGFILE"
        sleep "$RETRY_INTERVAL"
    done
}

# ===== MAIN LOOP =====
wait_for_ip_and_ping

while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    echo "[$TIMESTAMP] Starting iperf3 server on $SERVER_IP:$PORT..." | tee -a "$LOGFILE"

    # Fix /dev/urandom (OpenWrt workaround)
    if [ ! -c /dev/urandom ]; then
        echo "[$(date +"%Y-%m-%d %H:%M:%S")] /dev/urandom missing. Recreating..." | tee -a "$LOGFILE"
        rm -f /dev/urandom
        mknod -m 666 /dev/urandom c 1 9
    fi

    # Run iperf3 and stream output to both terminal and log
    OUTPUT=$(iperf3 -s -B "$SERVER_IP" -p "$PORT" 2>&1 | tee -a "$LOGFILE")

    echo "$OUTPUT" | grep -q "failed to read /dev/urandom"
    if [ $? -eq 0 ]; then
        echo "[$(date +"%Y-%m-%d %H:%M:%S")] Detected urandom error. Restarting in ${RETRY_INTERVAL}s..." | tee -a "$LOGFILE"
        sleep "$RETRY_INTERVAL"
        wait_for_ip_and_ping
    else
        echo "[$(date +"%Y-%m-%d %H:%M:%S")] iperf3 exited cleanly. Script ending." | tee -a "$LOGFILE"
        break
    fi
done
l