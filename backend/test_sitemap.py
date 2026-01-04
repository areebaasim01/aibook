#!/usr/bin/env python3
"""
Test script to verify sitemap functionality works correctly.
"""

import sys
from pathlib import Path

# Add the backend/src directory to the Python path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.utils import extract_urls_from_sitemap, extract_urls_from_sitemap_index


def test_sitemap_extraction():
    """
    Test the sitemap extraction functionality with the provided sitemap URL.
    """
    sitemap_url = "https://aibook-wrcb.vercel.app/docs/robotic-nervous-system/sitemap.xml"

    print(f"Testing sitemap extraction from: {sitemap_url}")

    urls = extract_urls_from_sitemap(sitemap_url)

    print(f"Found {len(urls)} URLs in the sitemap:")
    for i, url in enumerate(urls, 1):
        print(f"  {i}. {url}")

    return urls


if __name__ == "__main__":
    test_sitemap_extraction()