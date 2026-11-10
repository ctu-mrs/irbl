#!/bin/bash
# N-pair version (env.sh N_PAIRS): `pair <k>` first picks which UGV + UAV pair (mission node
# /ugv_uav_mission_<n>) gets the shelves; without it, pair 1. `abort` without a pair stops every pair.
#
# The UGV drives to the aisle position and stops; the UAV takes off from it, inspects the shelf and
# hovers where it leaves the aisle; the UGV drives below it, the UAV lands on it (precision landing),
# disarms and is re-attached; then the UGV carries it to the next shelf, or back to where it started.
#
#   ./deploy.sh [pair <n>] row <r> column <c> <left|right>   e.g.  ./deploy.sh pair 2 row 3 column 3 right
#   ./deploy.sh row <r> column <c> <left|right>   e.g.  ./deploy.sh row 5 column 2 left   (pair 1)
#   ./deploy.sh <r> <c> <left|right>              same, short form: ./deploy.sh 5 2 left
#   ./deploy.sh row 5 column 2 left  row 3 column 3 right  ...   several shelves in sequence
#   ./deploy.sh 5 2 left 3 3 right 1 4 left                    (same, short form)
#   ./deploy.sh xy <shelf_x> <shelf_y> <left|right> [<x> <y> <side> ...]   raw gz world coordinates
#   ./deploy.sh map                               print the rows / columns of the map
#   ./deploy.sh abort                             every pair holds where it is
#   ./deploy.sh pair 2 abort                      only pair 2
#
# row = aisle (1 = most -y), column = aisle segment between cross-aisles (1 = most -x), left = the
# shelf on the +y side of the aisle. Both come from config/warehouse_map.yaml (map.aisle_y,
# map.segment_x_min / segment_x_max); the UGV stops in the middle of that segment.
cd "$(dirname "$(readlink -f "$0")")"
. ./env.sh
MAP=./config/warehouse_map.yaml

# map.<key> list from the yaml, space separated
maplist() {
  python3 -c "import sys,yaml; m=yaml.safe_load(open('$MAP'))['/**']['ros__parameters']['map']; print(*m['$1'])"
}

usage() { sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; }

showmap() {
  local ys=($(maplist aisle_y)) x0=($(maplist segment_x_min)) x1=($(maplist segment_x_max)) i
  echo "rows (aisles):"
  for i in "${!ys[@]}"; do echo "  row $((i + 1)): y = ${ys[$i]}"; done
  echo "columns (segments):"
  for i in "${!x0[@]}"; do echo "  column $((i + 1)): x = ${x0[$i]} .. ${x1[$i]}"; done
}

check_side() {
  case "$1" in left|right) ;; *) echo "side must be 'left' or 'right', not '$1'" >&2; exit 1 ;; esac
}

# shelves collected as parallel lists, then one ~/deploy_sequence call
XS=() YS=() SIDES=()
add() {  # x y side (x / y as floats: the service fields are float64[])
  check_side "$3"
  local f
  f=$(python3 -c "import sys; print(float(sys.argv[1]), float(sys.argv[2]))" "$1" "$2") ||
    { echo "not a number: '$1' / '$2'" >&2; exit 1; }
  XS+=("${f% *}"); YS+=("${f#* }"); SIDES+=("'$3'")
}

call_deploy() {
  local i
  for i in "${!XS[@]}"; do echo "shelf $((i + 1)): x=${XS[$i]} y=${YS[$i]}, ${SIDES[$i]//\'/} side"; done
  local IFS=,
  echo "pair $PAIR (ugv$PAIR + uav$PAIR)"
  ros2 service call /ugv_uav_mission_$PAIR/deploy_sequence rbl_mission/srv/DeploySequence \
    "{shelf_x: [${XS[*]}], shelf_y: [${YS[*]}], side: [${SIDES[*]}]}"
}

PAIR=
case "$1" in
  pair|p) PAIR=$2; shift 2 ;;
  pair[0-9]*|p[0-9]*) PAIR=${1#pair}; PAIR=${PAIR#p}; shift ;;
esac
[[ -z "$PAIR" || ( "$PAIR" =~ ^[0-9]+$ && PAIR -ge 1 && PAIR -le N_PAIRS ) ]] || { echo "pair must be 1..$N_PAIRS, not '$PAIR'" >&2; exit 1; }

case "$1" in
  ""|-h|--help) usage; exit 0 ;;
  abort)
    for p in ${PAIR:-$(seq 1 $N_PAIRS)}; do ros2 service call /ugv_uav_mission_$p/abort std_srvs/srv/Trigger; done; exit $? ;;
  map) showmap; exit 0 ;;
  xy)
    shift
    (( $# > 0 && $# % 3 == 0 )) || { echo "usage: ./deploy.sh xy <shelf_x> <shelf_y> <left|right> [...]" >&2; exit 1; }
    while (( $# )); do add "$1" "$2" "$3"; shift 3; done
    call_deploy; exit $? ;;
esac
PAIR=${PAIR:-1}

# row / column form: drop the optional words "row" / "column" / "col"
args=()
for a in "$@"; do
  case "$a" in row|aisle|column|col|segment|,|then) ;; *) args+=("$a") ;; esac
done
(( ${#args[@]} > 0 && ${#args[@]} % 3 == 0 )) || { usage >&2; exit 1; }

ys=($(maplist aisle_y)) x0=($(maplist segment_x_min)) x1=($(maplist segment_x_max))
for ((k = 0; k < ${#args[@]}; k += 3)); do
  row=${args[$k]} col=${args[$((k + 1))]} side=${args[$((k + 2))]}
  [[ "$row" =~ ^[0-9]+$ && "$col" =~ ^[0-9]+$ ]] || { echo "row and column must be numbers (see ./deploy.sh map)" >&2; exit 1; }
  if (( row < 1 || row > ${#ys[@]} )); then
    echo "row $row does not exist: rows 1..${#ys[@]} (./deploy.sh map)" >&2; exit 1
  fi
  if (( col < 1 || col > ${#x0[@]} )); then
    echo "column $col does not exist: columns 1..${#x0[@]} (./deploy.sh map)" >&2; exit 1
  fi
  x=$(python3 -c "print(round((${x0[$((col - 1))]} + ${x1[$((col - 1))]}) / 2, 2))")
  echo "row $row, column $col, $side shelf"
  add "$x" "${ys[$((row - 1))]}" "$side"
done
call_deploy
