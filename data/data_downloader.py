
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
# check if the data already exists
#---------------------------------------------------------

def check_data_exists():
    """Check if the test data has already been downloaded."""
    good_path = DATA_DIR / GOOD_FILENAME
    bad_path = DATA_DIR / BAD_FILENAME

    if good_path.exists():
        print("\nTest data already exists:")
        print(f"  {good_path}")
        return True

    if bad_path.exists():
        print("\nTest data already exists but needs filename fixing:")
        print(f"  {bad_path}")
        return True

    print("\nTest data not found. Downloading...")
    return False



# ---------------------------------------------------------
# Download Rocket Test Data
# ---------------------------------------------------------

def download_test_data():
    print("\n========================================")
    print("Downloading Rocket Test Data")
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

    if  check_data_exists():
        print("========================================")
        print("Test Data Already Exists")
        print("========================================")
        print("\nWould you like to download the test data again? (y/n)")
        answer = input()
        if answer.lower() == "y":
            print("\n")

        else:
            print("\nexiting")
            return

    download_test_data()

    fix_filename()

    print("========================================")
    print("Test Data Ready")
    print("========================================")
    print("\nYou can now run the tests with: pio test -e native-for_mac_and_windows")


if __name__ == "__main__":
    main()
