#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
out=build/exploration_sim
mkdir -p "$out"
cc_cmd=${CC:-cc}
flags=(-std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Icommon/exploration)
rename=()
for symbol in default_config reset workspace_bytes guard_acceleration guard_progress begin step result apply_result note_goal_reached decide; do
  rename+=("-Dnf_exploration_${symbol}=nf_reference_${symbol}")
done
"$cc_cmd" "${flags[@]}" -DNF_EXPLORATION_DISABLE_NAV_PRUNING=1 "${rename[@]}" \
  -c common/exploration/exploration_policy.c -o "$out/policy_full_reference.o"
"$cc_cmd" "${flags[@]}" -DNF_EXPLORATION_TEST_REFERENCE \
  common/exploration/exploration_policy.c tools/solver_host/exploration_policy_test.c \
  "$out/policy_full_reference.o" -o "$out/exploration_policy_test"
"$out/exploration_policy_test"
