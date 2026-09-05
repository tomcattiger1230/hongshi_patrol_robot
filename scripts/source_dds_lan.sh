#!/usr/bin/env sh
# Generate and activate a Fast DDS profile restricted to the LAN-facing address.

robot320_peer="${1:-192.168.0.218}"
robot320_repo="${ROBOT320_REPO:-$(pwd)}"
robot320_template="${robot320_repo}/config/dds/fastdds_lan.xml.in"

if [ "$(uname -s)" = "Darwin" ]; then
  robot320_runtime_dir="${ROBOT320_DDS_RUNTIME_DIR:-${HOME}/Develop/fastdds-python/runtime}"
  robot320_interface="$(route -n get "${robot320_peer}" 2>/dev/null | awk '/interface:/{print $2; exit}')"
  robot320_address="$(ipconfig getifaddr "${robot320_interface}" 2>/dev/null)"
else
  robot320_runtime_dir="${ROBOT320_DDS_RUNTIME_DIR:-${robot320_repo}/.ros-deps/dds}"
  robot320_address="$(ip -4 route get "${robot320_peer}" 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')"
fi

if [ -z "${robot320_address}" ] || [ ! -f "${robot320_template}" ]; then
  echo "error: cannot determine the LAN address or locate ${robot320_template}" >&2
  return 2
fi

mkdir -p "${robot320_runtime_dir}"
robot320_profile="${robot320_runtime_dir}/fastdds_lan.xml"
sed "s/@ROBOT320_LAN_ADDRESS@/${robot320_address}/g" "${robot320_template}" >"${robot320_profile}"
export FASTDDS_DEFAULT_PROFILES_FILE="${robot320_profile}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

echo "Fast DDS LAN ready: address=${robot320_address} peer=${robot320_peer} domain=${ROS_DOMAIN_ID}"

unset robot320_address robot320_interface robot320_peer robot320_profile
unset robot320_repo robot320_runtime_dir robot320_template
