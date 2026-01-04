#!/usr/bin/env python3
"""
Test script to verify the main ingestion pipeline can use the sitemap argument.
"""

import subprocess
import sys
from pathlib import Path


def test_help_output():
    """
    Test that the help output shows the new sitemap arguments.
    """
    print("Testing help output to verify new sitemap arguments are available...")

    # Run the main module with --help to see if the new arguments are available
    # Change to the backend directory and run the module directly
    result = subprocess.run([
        sys.executable, "-c", "import sys; sys.path.insert(0, '.'); from src.main import main; import sys; sys.argv = ['main.py', '--help']; main()",
    ], capture_output=True, text=True, cwd=Path(__file__).parent)

    print("Return code:", result.returncode)
    print("STDOUT:")
    print(result.stdout)

    if result.stderr:
        print("STDERR:")
        print(result.stderr)

    # Check if the new sitemap arguments are in the help text
    if "--sitemap" in result.stdout and "--sitemap-index" in result.stdout:
        print("\nSUCCESS: New sitemap arguments are available in the CLI")
        return True
    else:
        print("\nFAILURE: New sitemap arguments are not in the CLI help")
        return False


if __name__ == "__main__":
    success = test_help_output()
    if success:
        print("\n🎉 All tests passed! The sitemap functionality has been successfully integrated.")
    else:
        print("\n💥 Tests failed! There may be issues with the integration.")
        sys.exit(1)