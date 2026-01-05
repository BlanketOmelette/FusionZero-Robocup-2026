#!/usr/bin/env bash
set -e

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set. You are not in an X11-forwarded SSH session."
  echo "If using Windows, start VcXsrv/Xming and connect with X11 forwarding enabled."
  exit 1
fi

echo "DISPLAY=$DISPLAY"
echo "Launching xclock (close the window to return)..."
xclock
