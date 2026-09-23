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
