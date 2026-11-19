"""Print a signing Team ID using local Xcode metadata and public certificates.

No Apple ID password, private key export, or authentication-token access is used.
"""
import argparse
import base64
import datetime
import hashlib
import os
from pathlib import Path
import plistlib
import re
import subprocess
import sys

TEAM_ID = re.compile(r"^[A-Z0-9]{10}$")


def run(arguments, data=None):
    result = subprocess.run(arguments, input=data, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, check=False, timeout=30)
    return result.stdout if result.returncode == 0 else b""


def account_teams():
    path = Path.home() / "Library/Preferences/com.apple.dt.Xcode.plist"
    try:
        with path.open("rb") as source:
            # Inspect only provisioning-team metadata, never account credentials.
            groups = plistlib.load(source).get("IDEProvisioningTeamByIdentifier", {})
    except (OSError, ValueError, plistlib.InvalidFileException):
        return {}
    teams = {}
    for group in groups.values():
        if not isinstance(group, list):
            continue
        for item in group:
            identifier = item.get("teamID", "") if isinstance(item, dict) else ""
            if TEAM_ID.fullmatch(identifier):
                teams[identifier] = item.get("teamName", identifier)
    return teams


def certificate_teams():
    identities = run(["/usr/bin/security", "find-identity", "-v", "-p", "codesigning"])
    valid = set(re.findall(rb"\b[0-9A-F]{40}\b", identities))
    teams = {}
    for name in ("Apple Development", "Apple Distribution", "iPhone Developer"):
        certificates = run(["/usr/bin/security", "find-certificate", "-a", "-c", name, "-p"])
        for pem in re.findall(rb"-----BEGIN CERTIFICATE-----(.*?)-----END CERTIFICATE-----", certificates, re.S):
            certificate = base64.b64decode(pem)
            digest = hashlib.sha1(certificate).hexdigest().upper().encode()
            if digest not in valid:
                continue
            subject = run(["/usr/bin/openssl", "x509", "-inform", "DER", "-noout",
                           "-subject", "-nameopt", "RFC2253"], certificate).decode()
            # The certificate CN suffix can be a person ID; OU is the Team ID.
            match = re.search(r"(?:^|[, ])OU=([A-Z0-9]{10})(?:,|$)", subject.strip())
            if match:
                teams[match.group(1)] = "Installed signing identity"
    return teams


def profile_teams():
    directories = ("Library/Developer/Xcode/UserData/Provisioning Profiles",
                   "Library/MobileDevice/Provisioning Profiles")
    teams = {}
    now = datetime.datetime.now(datetime.timezone.utc)
    for directory in directories:
        for path in (Path.home() / directory).glob("*.mobileprovision"):
            data = run(["/usr/bin/security", "cms", "-D", "-i", str(path)])
            try:
                profile = plistlib.loads(data)
            except (ValueError, plistlib.InvalidFileException):
                continue
            expiration = profile.get("ExpirationDate")
            if expiration and expiration.replace(tzinfo=datetime.timezone.utc) < now:
                continue
            for identifier in profile.get("TeamIdentifier", []):
                if TEAM_ID.fullmatch(identifier):
                    teams[identifier] = profile.get("TeamName", identifier)
    return teams


def choose_team(accounts, certificates, profiles):
    if accounts:
        signed_in = accounts.keys() & certificates.keys()
        candidates = {key: accounts[key] for key in signed_in} if signed_in else accounts
    else:
        candidates = certificates or profiles
    if len(candidates) == 1:
        return next(iter(candidates))
    if not candidates:
        raise ValueError("No Apple signing team found. Add your developer account in Xcode Settings > Accounts, then retry.")
    choices = ", ".join(f"{identifier} ({name})" for identifier, name in sorted(candidates.items()))
    raise ValueError(f"Multiple signing teams found: {choices}. Set APPLE_TEAM_ID explicitly to select one.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="Show discovered Team IDs and names")
    args = parser.parse_args()
    explicit = os.environ.get("APPLE_TEAM_ID", "")
    if explicit and explicit not in ("YOUR_TEAM_ID", "YOUR_APPLE_TEAM_ID"):
        if not TEAM_ID.fullmatch(explicit):
            raise ValueError("APPLE_TEAM_ID must be a ten-character Apple Team ID")
        print(explicit)
        return
    if sys.platform != "darwin":
        raise ValueError("Automatic signing-team detection requires macOS")
    accounts, certificates = account_teams(), certificate_teams()
    profiles = profile_teams() if not accounts and not certificates else {}
    if args.list:
        for identifier, name in sorted({**profiles, **certificates, **accounts}.items()):
            print(f"{identifier}\t{name}")
    else:
        identifier = choose_team(accounts, certificates, profiles)
        print(identifier)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.TimeoutExpired) as error:
        print(f"Apple signing: {error}", file=sys.stderr)
        sys.exit(1)
