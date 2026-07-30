#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/../.." && pwd)"

expected_branch="integration/d-task-final"
integration_start="a5978ef65ec004b79c7ca7cb41011951adfdfe85"
expected_ros="jazzy"
expected_px4_msgs_branch="release/1.16"
expected_px4_msgs_commit="392e831c1f659429ca83902e66820d7094591410"
px4_msgs_dir="${workspace_root}/src/px4_msgs"
failed=0

pass() { printf 'PASS: %s\n' "$1"; }
fail() { printf 'FAIL: %s\n' "$1" >&2; failed=1; }

current_branch="$(git -C "${workspace_root}" branch --show-current)"
current_commit="$(git -C "${workspace_root}" rev-parse HEAD)"
printf 'Git branch: %s\nGit commit: %s\n' "${current_branch}" "${current_commit}"

if [[ "${current_branch}" == "${expected_branch}" ]]; then
  pass "current branch is ${expected_branch}"
else
  fail "current branch is ${current_branch}, expected ${expected_branch}"
fi

if git -C "${workspace_root}" merge-base --is-ancestor \
    "${integration_start}" "${current_commit}"; then
  pass "integration start commit is an ancestor of HEAD"
else
  fail "HEAD does not descend from ${integration_start}"
fi

if [[ "${ROS_DISTRO:-}" == "${expected_ros}" ]]; then
  pass "ROS_DISTRO is ${expected_ros}"
else
  fail "ROS_DISTRO is ${ROS_DISTRO:-unset}, expected ${expected_ros}"
fi

if [[ -d "${px4_msgs_dir}/.git" ]] ||
   git -C "${px4_msgs_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  px4_msgs_branch="$(git -C "${px4_msgs_dir}" branch --show-current)"
  px4_msgs_commit="$(git -C "${px4_msgs_dir}" rev-parse HEAD)"
  printf 'px4_msgs branch: %s\npx4_msgs commit: %s\n' \
    "${px4_msgs_branch:-DETACHED}" "${px4_msgs_commit}"

  if [[ "${px4_msgs_commit}" == "${expected_px4_msgs_commit}" ]]; then
    pass "px4_msgs commit matches the frozen revision"
  else
    fail "px4_msgs commit does not match ${expected_px4_msgs_commit}"
  fi

  if [[ -z "${px4_msgs_branch}" ||
        "${px4_msgs_branch}" == "${expected_px4_msgs_branch}" ]]; then
    pass "px4_msgs is release/1.16 or detached at the frozen commit"
  else
    fail "px4_msgs branch is ${px4_msgs_branch}, expected ${expected_px4_msgs_branch}"
  fi
else
  fail "src/px4_msgs is absent; import dependencies/px4_msgs.repos before a full build"
fi

if [[ -n "${PX4_VERSION:-}" &&
      "${PX4_VERSION}" != "v1.16.0" &&
      "${PX4_VERSION}" != "1.16.0" ]]; then
  fail "PX4_VERSION=${PX4_VERSION} conflicts with the frozen v1.16.0 baseline"
else
  pass "no environment override selects PX4 v1.15 or v1.17"
fi

bad_baseline="$(
  grep -RniE \
    '(current|当前|唯一).*(baseline|基线).*(v?1\.(15|17))|v?1\.(15|17).*(current|当前|唯一).*(baseline|基线)' \
    "${workspace_root}/dependencies" \
    "${workspace_root}/docs/integration_branch_map.md" 2>/dev/null || true
)"
if [[ -z "${bad_baseline}" ]]; then
  pass "integration baseline files do not select PX4 v1.15 or v1.17"
else
  printf '%s\n' "${bad_baseline}" >&2
  fail "integration files contain a forbidden current PX4 baseline"
fi

colcon_output="$(cd "${workspace_root}" && colcon list)"
printf '%s\n' "${colcon_output}"
duplicate_packages="$(
  printf '%s\n' "${colcon_output}" |
    awk 'NF {print $1}' |
    sort |
    uniq -d
)"
if [[ -z "${duplicate_packages}" ]]; then
  pass "colcon package names are unique"
else
  printf 'Duplicate package names:\n%s\n' "${duplicate_packages}" >&2
  fail "duplicate ROS packages detected"
fi

if (( failed != 0 )); then
  exit 1
fi
pass "all baseline checks passed"
