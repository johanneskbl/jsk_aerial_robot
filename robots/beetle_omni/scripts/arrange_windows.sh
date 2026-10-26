#!/usr/bin/env bash
# Put RViz maximized on the secondary monitor, then Gazebo as a small window raised on top
# of it in that monitor's top-left corner. Needs X11 and wmctrl: `sudo apt install wmctrl`.
#
# Options:
#   --monitor NAME     xrandr monitor to use (default: first non-primary monitor)
#   --gazebo-size WxH  Gazebo window size in px (default: 2/3 of the monitor width, 16:9)
#   --timeout SEC      stop waiting for the windows after SEC seconds (default: 120)

monitor=""
gazebo_size=""
timeout=120
while [[ $# -gt 0 ]]; do
  case "$1" in
    --monitor) monitor="$2"; shift 2 ;;
    --gazebo-size) gazebo_size="$2"; shift 2 ;;
    --timeout) timeout="$2"; shift 2 ;;
    *) shift ;;  # __name:= / __log:= added by roslaunch
  esac
done

log() { echo "[arrange_windows] $*"; }

if ! command -v wmctrl > /dev/null || ! wmctrl -m > /dev/null 2>&1; then
  log "wmctrl not installed or no X11 window manager found, leaving the windows where they are"
  exit 0
fi

# "xrandr --listmonitors" lines look like " 1: +HDMI-0 2880/597x5120/336+0+0  HDMI-0", '*' marks the primary
geometry=$(xrandr --listmonitors | awk -v want="$monitor" 'NR > 1 {
  name = $2; gsub(/[+*]/, "", name)
  if (want == "" ? $2 !~ /\*/ : name == want) { print $3; exit }
}')
re='^([0-9]+)/[0-9]+x[0-9]+/[0-9]+\+(-?[0-9]+)\+(-?[0-9]+)$'
if [[ ! $geometry =~ $re ]]; then
  log "no ${monitor:-secondary} monitor found, leaving the windows where they are"
  exit 0
fi
mon_w=${BASH_REMATCH[1]} mon_x=${BASH_REMATCH[2]} mon_y=${BASH_REMATCH[3]}

if [[ $gazebo_size =~ ^([0-9]+)x([0-9]+)$ ]]; then
  gz_w=${BASH_REMATCH[1]} gz_h=${BASH_REMATCH[2]}
else
  gz_w=$((mon_w * 2 / 3)) gz_h=$((mon_w * 2 / 3 * 9 / 16))
fi

# Print the id of the first window whose WM_CLASS matches $1 and whose title contains $2
find_window() {
  wmctrl -lx | awk -v cls="$1" -v title="$2" 'tolower($3) ~ cls && index($0, title) { print $1; exit }'
}

place_rviz() {
  sleep 0.5  # let the window finish mapping before moving it
  wmctrl -i -r "$1" -b remove,maximized_vert,maximized_horz
  # Maximize applies to the monitor the window is on, so move it there first
  wmctrl -i -r "$1" -e "0,$((mon_x + 50)),$((mon_y + 50)),-1,-1"
  wmctrl -i -r "$1" -b add,maximized_vert,maximized_horz
  log "RViz maximized on monitor at +${mon_x}+${mon_y}"
}

place_gazebo() {
  sleep 0.5
  wmctrl -i -r "$1" -b remove,maximized_vert,maximized_horz
  wmctrl -i -r "$1" -e "0,${mon_x},${mon_y},${gz_w},${gz_h}"
  # Raise once (no "above" state), so clicking RViz brings it back to the front
  wmctrl -i -a "$1"
  log "Gazebo ${gz_w}x${gz_h} in the top-left corner"
}

rviz_id=""
for ((i = 0; i < 2 * timeout; i++)); do
  if [[ -z $rviz_id ]]; then
    rviz_id=$(find_window '^rviz[.]' 'RViz')
    [[ -n $rviz_id ]] && place_rviz "$rviz_id"
  else
    gazebo_id=$(find_window '^(gazebo|gzclient)[.]' 'Gazebo')
    if [[ -n $gazebo_id ]]; then
      place_gazebo "$gazebo_id"
      exit 0
    fi
  fi
  sleep 0.5
done

if [[ -z $rviz_id ]]; then
  log "timed out waiting for the RViz window"
else
  log "timed out waiting for the Gazebo window"
fi
