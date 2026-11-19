#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
: "${1:?Usage: bash scripts/export-ipa.sh /absolute/path/ExportOptions.local.plist}"
xcodebuild -exportArchive -archivePath ios/build/BrachiFlex.xcarchive \
    -exportPath ios/build/ipa -exportOptionsPlist "$1" -allowProvisioningUpdates
