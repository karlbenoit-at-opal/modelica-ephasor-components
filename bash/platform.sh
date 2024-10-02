#!/bin/echo Usage: source platform.sh

DETECTED_PLATFORM="unknown"
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

if [[ "$OSTYPE" == "msys" ]]; then
    DETECTED_PLATFORM="Windows"
    source "${SCRIPT_DIR}/bash/windows.sh"
elif [[ "$OSTYPE" == "cygwin" ]]; then
    DETECTED_PLATFORM="Windows"
    source "${SCRIPT_DIR}/bash/windows.sh"
elif [[ "$OSTYPE" == "linux-gnu"* ]] ||
        [[ "$OSTYPE" == "darwin"* ]] ||
        [[ "$OSTYPE" == "freebsd"* ]]; then
    DETECTED_PLATFORM="Linux"
    source "${SCRIPT_DIR}/bash/linux.sh"
else
	echo -e "Detected unknown platform."
	exit 1
fi
