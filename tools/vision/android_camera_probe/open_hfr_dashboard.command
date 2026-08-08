#!/bin/zsh
set -euo pipefail

dashboard_dir=${0:A:h}
cd "${dashboard_dir}/../../.."
exec python3 "${dashboard_dir}/hfr_dashboard.py"
