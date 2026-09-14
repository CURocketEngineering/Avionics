
import json
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path


# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

REPO_API = (
    "https://api.github.com/repos/"
    "CURocketEngineering/Rocket-Test-Data/releases/tags/v1.0.0"
)

DATA_DIR = Path("data")

BAD_FILENAME = "AA.Data.Collection.-.Second.Launch.Trimmed.csv"
GOOD_FILENAME = "AA Data Collection - Second Launch Trimmed.csv"


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------

def run_command(command):
    """Run a command and stop if it fails."""
    print(f"\n> {' '.join(command)}")
    result = subprocess.run(command)

    if result.returncode != 0:
        print(f"\nCommand failed with exit code {result.returncode}")
        sys.exit(result.returncode)


# ---------------------------------------------------------
# Download Rocket Test Data
# ---------------------------------------------------------

def download_test_data():
    print("\n========================================")
    print("Downloading Rocket Test Data v1.0.0")
    print("========================================")

    DATA_DIR.mkdir(parents=True, exist_ok=True)

    request = urllib.request.Request(
        REPO_API,
        headers={
            "Accept": "application/vnd.github+json",
            "User-Agent": "Avionics-Test-Runner",
        },
    )

    try:
        with urllib.request.urlopen(request) as response:
            release = json.load(response)
    except Exception as e:
        print(f"Failed to get GitHub release information: {e}")
        sys.exit(1)

    assets = release.get("assets", [])

    if not assets:
        print("No release assets found.")
        sys.exit(1)

    for asset in assets:
        filename = asset["name"]
        download_url = asset["browser_download_url"]

        output_path = DATA_DIR / filename

        print(f"\nDownloading:")
        print(f"  {download_url}")
        print(f"  -> {output_path}")

        try:
            download_request = urllib.request.Request(
                download_url,
                headers={
                    "User-Agent": "Avionics-Test-Runner",
                },
            )

            with urllib.request.urlopen(download_request) as response:
                with open(output_path, "wb") as output:
                    shutil.copyfileobj(response, output)

        except Exception as e:
            print(f"Failed to download {filename}: {e}")
            sys.exit(1)


# ---------------------------------------------------------
# Fix Filename
# ---------------------------------------------------------

def fix_filename():
    print("\n========================================")
    print("Fixing Test Data Filename")
    print("========================================")

    old_path = DATA_DIR / BAD_FILENAME
    new_path = DATA_DIR / GOOD_FILENAME

    if not old_path.exists():
        # It may already have been renamed.
        if new_path.exists():
            print(f"Already fixed:")
            print(f"  {new_path}")
            return

        print(f"Could not find:")
        print(f"  {old_path}")
        sys.exit(1)

    if new_path.exists():
        print(f"Removing existing:")
        print(f"  {new_path}")
        new_path.unlink()

    print(f"Moving:")
    print(f"  {old_path}")
    print(f"  -> {new_path}")

    old_path.rename(new_path)




# ---------------------------------------------------------
# Main
# ---------------------------------------------------------

def main():
    print("========================================")
    print("CURE Avionics Native Test Runner")
    print("========================================")

    download_test_data()
    fix_filename()
    print("========================================")
    print("Data Downloaded and Renamed")
    print("========================================")


if __name__ == "__main__":
    main()
