#!/usr/bin/env bash
set -euo pipefail

# Keep traffic whose source is the SIM-side Ethernet address on that uplink even
# while Wi-Fi owns the lower-metric default route.  Table numbers are used
# directly so this does not require modifying /etc/iproute2/rt_tables.
SIM_INTERFACE="${SIM_INTERFACE:-enp85s0}"
SIM_ADDRESS="${SIM_ADDRESS:-192.168.1.50}"
SIM_GATEWAY="${SIM_GATEWAY:-192.168.1.1}"
SIM_SUBNET="${SIM_SUBNET:-192.168.1.0/24}"
SIM_TABLE="${SIM_TABLE:-102}"
SIM_RULE_PRIORITY="${SIM_RULE_PRIORITY:-1020}"

case "${1:-start}" in
  start)
    # network-online is not sufficient during a live netplan/NetworkManager
    # reload: the device can briefly exist without its static address.
    address_ready=false
    for _attempt in $(seq 1 30); do
      if ip -4 -o address show dev "${SIM_INTERFACE}" | grep -q "inet ${SIM_ADDRESS}/"; then
        address_ready=true
        break
      fi
      sleep 1
    done
    if [[ "${address_ready}" != true ]]; then
      echo "${SIM_ADDRESS} is not configured on ${SIM_INTERFACE}" >&2
      exit 1
    fi
    ip route replace "${SIM_SUBNET}" dev "${SIM_INTERFACE}" src "${SIM_ADDRESS}" table "${SIM_TABLE}"
    ip route replace default via "${SIM_GATEWAY}" dev "${SIM_INTERFACE}" table "${SIM_TABLE}"
    while ip rule del priority "${SIM_RULE_PRIORITY}" 2>/dev/null; do :; done
    ip rule add priority "${SIM_RULE_PRIORITY}" from "${SIM_ADDRESS}/32" table "${SIM_TABLE}"
    ip route flush cache
    ;;
  stop)
    while ip rule del priority "${SIM_RULE_PRIORITY}" 2>/dev/null; do :; done
    ip route flush table "${SIM_TABLE}"
    ip route flush cache
    ;;
  *)
    echo "Usage: $0 [start|stop]" >&2
    exit 2
    ;;
esac
