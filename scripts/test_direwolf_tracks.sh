#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "$0")/.." && pwd)

for tool in ffmpeg atest; do
    command -v "$tool" >/dev/null || { echo "error: $tool is required" >&2; exit 1; }
done

if (($#)); then
    tracks=("$@")
else
    tracks=(
        "$repo_root/test/fixtures/01_40-Mins-Traffic-on-144.39.flac"
        "$repo_root/test/fixtures/02_100-Mic-E-Bursts-DE-emphasized.flac"
    )
fi

work_dir=$(mktemp -d "${TMPDIR:-/tmp}/esp32-afsk-direwolf.XXXXXX")
trap 'rm -rf "$work_dir"' EXIT

for track in "${tracks[@]}"; do
    wav="$work_dir/$(basename "$track").wav"

    # atest requires WAV without ffmpeg's usual LIST metadata chunk.
    ffmpeg -nostdin -hide_banner -loglevel error \
        -i "$track" -map_metadata -1 -fflags +bitexact -flags:a +bitexact \
        -ac 1 -ar 48000 -c:a pcm_s16le -y "$wav"

    printf '\n=== %s ===\n' "$(basename "$track")"
    # -F 0 accepts only frames whose FCS was valid without bit recovery.
    atest -F 0 -B 1200 "$wav" 2>&1 | tail -n 2
done
