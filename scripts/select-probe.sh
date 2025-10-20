#!/usr/bin/env bash
set -euo pipefail

if ! command -v probe-rs >/dev/null 2>&1; then
  echo "probe-rs command not found. Install probe-rs-cli first." >&2
  exit 1
fi

entries=()
indices=()
ids=()

while IFS= read -r line; do
  trimmed="${line## }"
  if [[ -z "${trimmed}" ]]; then
    continue
  fi
  if [[ "${line}" =~ -- ]]; then
    entries+=("${line}")
    index=""
    if [[ "${line}" =~ ^\[([0-9]+)\]: ]]; then
      index="${BASH_REMATCH[1]}"
    elif [[ "${line}" =~ ^([0-9]+): ]]; then
      index="${BASH_REMATCH[1]}"
    fi
    if [[ -n "${index}" ]] && [[ "${line}" =~ --[[:space:]]*([0-9a-fA-F]{4}:[0-9a-fA-F]{4}(:[^[:space:]]+)?) ]]; then
      indices+=("${index}")
      ids+=("${BASH_REMATCH[1]}")
    fi
  fi
done < <(probe-rs list)

if [[ ${#entries[@]} -eq 0 ]]; then
  echo "No debug probes detected. Connect a probe and retry." >&2
  exit 1
fi

echo "Detected debug probes:" >&2
for entry in "${entries[@]}"; do
  echo "  ${entry}" >&2
done

echo >&2
read -r -p "Select probe index or enter VID:PID[:SERIAL]: " choice

if [[ -z "${choice}" ]]; then
  echo "Selection cancelled." >&2
  exit 1
fi

lookup_id() {
  local target="$1"
  local i=0
  local count="${#indices[@]}"
  while [[ $i -lt $count ]]; do
    if [[ "${indices[$i]}" == "${target}" ]]; then
      echo "${ids[$i]}"
      return 0
    fi
    i=$((i + 1))
  done
  return 1
}

selected=""

if [[ "${choice}" =~ ^[0-9]+$ ]]; then
  if selected=$(lookup_id "${choice}"); then
    true
  else
    selected=""
  fi
  if [[ -z "${selected}" ]]; then
    echo "Could not resolve probe with index ${choice}." >&2
    exit 1
  fi
else
  selected="${choice}"
fi

if [[ ! "${selected}" =~ ^[0-9a-fA-F]{4}:[0-9a-fA-F]{4}(:[^[:space:]]+)?$ ]]; then
  echo "Invalid probe identifier: ${selected}" >&2
  exit 1
fi

echo "export PROBE_ID=${selected}"
