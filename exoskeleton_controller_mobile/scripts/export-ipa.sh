#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ -z "${DEVELOPER_DIR:-}" && -d /Applications/Xcode.app/Contents/Developer ]]; then
    export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer
fi
options="${1:-}"
if [[ -z "$options" ]]; then
    APPLE_TEAM_ID="$(python3 scripts/detect_apple_team.py)"
    export APPLE_TEAM_ID
    mkdir -p ios/build
    options="$PWD/ios/build/ExportOptions.plist"
    python3 - "$options" <<'PY'
import os
import plistlib
import sys
with open(sys.argv[1], "wb") as output:
    plistlib.dump({"method": os.environ.get("APPLE_EXPORT_METHOD", "debugging"),
                  "signingStyle": "automatic", "teamID": os.environ["APPLE_TEAM_ID"]}, output)
PY
fi
xcodebuild -exportArchive -archivePath ios/build/BrachiFlex.xcarchive \
    -exportPath ios/build/ipa -exportOptionsPlist "$options" -allowProvisioningUpdates
