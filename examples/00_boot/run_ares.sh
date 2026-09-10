#!/bin/sh
# Runs this example in ares, from a CD image: builds the image if the
# executable is newer, then boots it with the PlayStation core. Set ARES
# to point at another build; the BIOS lives in ares' Firmware folder.
cd "$(dirname "$0")" || exit 1

name="$(basename "$PWD")"
make cd >/dev/null || exit 1

ARES="${ARES:-$HOME/Juegos/emuladores/ares/ares}"

exec "$ARES" --system "PlayStation" "$name.cue"
