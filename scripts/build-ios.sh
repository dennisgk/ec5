#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "$(uname -s)" != Darwin ]]; then
    echo "iOS builds require macOS and Xcode." >&2
    exit 1
fi
command -v xcodegen >/dev/null || { echo "Install XcodeGen: brew install xcodegen" >&2; exit 1; }
rustup target add aarch64-apple-ios
cargo build --release --locked --lib --target aarch64-apple-ios
(cd ios && xcodegen generate)
if [[ "${1:-}" == "--unsigned" ]]; then
    xcodebuild -project ios/BrachiFlex.xcodeproj -scheme BrachiFlex -configuration Release \
        -sdk iphoneos -destination 'generic/platform=iOS' -derivedDataPath ios/DerivedData \
        CODE_SIGNING_ALLOWED=NO build
    exit
fi
: "${APPLE_TEAM_ID:?Set APPLE_TEAM_ID to your Apple signing team ID}"
xcodebuild -project ios/BrachiFlex.xcodeproj -scheme BrachiFlex -configuration Release \
    -destination 'generic/platform=iOS' -archivePath ios/build/BrachiFlex.xcarchive \
    DEVELOPMENT_TEAM="$APPLE_TEAM_ID" -allowProvisioningUpdates archive
echo "Archive ready. Export in Xcode Organizer, or run scripts/export-ipa.sh with your ExportOptions plist."
