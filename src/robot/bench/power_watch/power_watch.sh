#!/bin/bash
# Log the Pi's power health to /home/pi/power.log — once at boot, then every
# 10s. Decodes vcgencmd get_throttled bits:
#   0  under-voltage now          16  under-voltage has occurred since boot
#   1  arm freq capped now        17  freq capping has occurred
#   2  throttled now              18  throttling has occurred
#   3  soft temp limit now        19  soft temp limit has occurred
# 0x0 = healthy. (The Pi cannot detect over-voltage — do not exceed 5.2V at
# the buck; that is enforced with a multimeter, not software.)

LOG=/home/pi/power.log

decode() {
    local hex=${1#*=}
    local val=$((hex))
    local msgs=()
    ((val & 0x1))     && msgs+=("UNDER-VOLTAGE-NOW")
    ((val & 0x2))     && msgs+=("FREQ-CAPPED-NOW")
    ((val & 0x4))     && msgs+=("THROTTLED-NOW")
    ((val & 0x8))     && msgs+=("SOFT-TEMP-LIMIT-NOW")
    ((val & 0x10000)) && msgs+=("under-voltage-since-boot")
    ((val & 0x20000)) && msgs+=("freq-capped-since-boot")
    ((val & 0x40000)) && msgs+=("throttled-since-boot")
    ((val & 0x80000)) && msgs+=("soft-temp-limit-since-boot")
    [ ${#msgs[@]} -eq 0 ] && echo "OK" || echo "${msgs[*]}"
}

echo "$(date -Is) BOOT ---- power watch started" >> "$LOG"

prev=""
while true; do
    raw=$(vcgencmd get_throttled)
    state=$(decode "$raw")
    # Always log at start and whenever the state changes; heartbeat every 10 min.
    now=$(date -Is)
    if [ "$state" != "$prev" ]; then
        echo "$now $raw $state" >> "$LOG"
        prev=$state
    elif [ $(( $(date +%s) % 600 )) -lt 10 ]; then
        echo "$now $raw $state (heartbeat)" >> "$LOG"
    fi
    sleep 10
done
