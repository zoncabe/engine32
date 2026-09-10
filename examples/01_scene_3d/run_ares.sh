#!/bin/sh
# Runs this example in ares, from a CD image: builds the image if the
# executable is newer, then boots it with the PlayStation core. Set ARES
# to point at another build; the BIOS lives in ares' Firmware folder.
cd "$(dirname "$0")" || exit 1

# Launched from a file manager there is no shell profile: the toolchain
# and mkpsxiso live here.
export PATH="$HOME/.local/psx/bin:$PATH"

name="$(basename "$PWD")"
make cd || exit 1

ARES="${ARES:-$HOME/Juegos/emuladores/ares/ares}"

exec "$ARES" --system "PlayStation" "$name.cue"
