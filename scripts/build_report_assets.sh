#!/usr/bin/env bash
# Regenerate the S3VO report's real-world figures and metrics from the trial bags.
#   build_report_assets.sh [report_dir]    (default: ~/Documents/TeX/s3vo_report_aerosub)
# Edit ../config/report_trials.yaml to change trials, windows, video sync or snapshots.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPORT="${1:-$HOME/Documents/TeX/s3vo_report_aerosub}"
BAGS="$HERE/../bags/bags_s3vo-20260925T142349Z-1-001 (Copy)/bags_s3vo"
DATA="$HERE/../data"

set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
set -u

TRIALS=$(python3 -c "import yaml;print(' '.join(t['bag'] for t in yaml.safe_load(open('$HERE/../config/report_trials.yaml'))['trials']))")
python3 "$HERE/extract_bags.py" "$BAGS" "$DATA" $TRIALS
python3 "$HERE/make_report.py" "$DATA" "$REPORT"
(cd "$REPORT" && latexmk -pdf -interaction=nonstopmode main.tex > /dev/null && echo "built $REPORT/main.pdf")
