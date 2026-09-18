#!/bin/sh
# Runs this example in PCSX-Redux: the AppImage the PSX.Dev extension for
# VS Code installs. Set PCSX_REDUX to point at another build.
cd "$(dirname "$0")" || exit 1

exe="$(basename "$PWD").ps-exe"
if [ ! -f "$exe" ]; then
	echo "$exe not found: build it first (make)"
	exit 1
fi

REDUX="${PCSX_REDUX:-$HOME/.config/Code/User/globalStorage/grumpycoders.psx-dev/PCSX-Redux-HEAD-x86_64.AppImage}"

exec "$REDUX" -run -exe "$exe"
