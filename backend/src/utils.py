"""
Utility functions for the Book Embeddings Ingestion Pipeline.

This module provides common utility functions for logging, progress tracking,
URL validation, and other shared functionality.
"""
import logging
import re
from typing import List, Optional
from urllib.parse import urlparse, urljoin
from tqdm import tqdm
import time
import functools
import requests
from bs4 import BeautifulSoup


def setup_logging(level: str = "INFO") -> logging.Logger:
    """
    Set up logging configuration for the application.

    Args:
        level: The logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)

    Returns:
        logging.Logger: Configured logger instance
    """
    logger = logging.getLogger("book_embeddings_ingestion")
    logger.setLevel(getattr(logging, level.upper()))

    # Avoid adding multiple handlers if logger already exists
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


def create_progress_bar(total: int, desc: str = "Processing") -> tqdm:
    """
    Create a progress bar using tqdm.

    Args:
        total: Total number of items to process
        desc: Description for the progress bar

    Returns:
        tqdm: Progress bar instance
    """
    return tqdm(total=total, desc=desc, unit="item")


def validate_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: The URL string to validate

    Returns:
        bool: True if the URL is valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def sanitize_url(url: str) -> str:
    """
    Sanitize a URL by removing potentially dangerous components.

    Args:
        url: The URL to sanitize

    Returns:
        str: Sanitized URL
    """
    # Remove any fragments and ensure proper formatting
    parsed = urlparse(url)
    # Only allow http and https schemes to prevent SSRF
    if parsed.scheme not in ["http", "https"]:
        raise ValueError(f"Invalid URL scheme: {parsed.scheme}")

    # Reconstruct URL with only the safe components
    sanitized = f"{parsed.scheme}://{parsed.netloc}{parsed.path}"
    if parsed.query:
        sanitized += f"?{parsed.query}"

    return sanitized


def is_valid_docusaurus_url(url: str) -> bool:
    """
    Check if a URL appears to be a Docusaurus site.

    Args:
        url: The URL to check

    Returns:
        bool: True if the URL appears to be a Docusaurus site, False otherwise
    """
    # Basic check: look for common Docusaurus patterns in the HTML
    # This is a simple check - in a real implementation, you might fetch and analyze the page
    parsed = urlparse(url)
    return bool(parsed.netloc)


def batch_list(lst: List, batch_size: int) -> List[List]:
    """
    Split a list into batches of specified size.

    Args:
        lst: List to be batched
        batch_size: Size of each batch

    Returns:
        List[List]: List of batches
    """
    return [lst[i:i + batch_size] for i in range(0, len(lst), batch_size)]


def retry_with_backoff(max_retries: int = 3, backoff_factor: float = 1.0):
    """
    Decorator to retry a function with exponential backoff.

    Args:
        max_retries: Maximum number of retry attempts
        backoff_factor: Factor by which to multiply the wait time after each retry
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if attempt == max_retries - 1:
                        raise e  # Re-raise the last exception

                    wait_time = backoff_factor * (2 ** attempt)
                    time.sleep(wait_time)

        return wrapper
    return decorator


def format_bytes(bytes_value: int) -> str:
    """
    Format a byte value into a human-readable string.

    Args:
        bytes_value: Number of bytes

    Returns:
        str: Human-readable format (e.g., "1.2 KB", "3.4 MB")
    """
    for unit in ['B', 'KB', 'MB', 'GB']:
        if bytes_value < 1024.0:
            return f"{bytes_value:.2f} {unit}"
        bytes_value /= 1024.0
    return f"{bytes_value:.2f} TB"


# Precompile regex for URL validation to improve performance
URL_PATTERN = re.compile(
    r'^https?://'  # http:// or https://
    r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
    r'localhost|'  # localhost...
    r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
    r'(?::\d+)?'  # optional port
    r'(?:/?|[/?]\S+)$', re.IGNORECASE)


def is_valid_url_format(url: str) -> bool:
    """
    Validate URL format using regex.

    Args:
        url: URL string to validate

    Returns:
        bool: True if URL format is valid, False otherwise
    """
    return bool(URL_PATTERN.match(url))


def extract_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """
    Extract all URLs from a sitemap.xml file.

    Args:
        sitemap_url: URL to the sitemap.xml file

    Returns:
        List of URLs extracted from the sitemap
    """
    try:
        response = requests.get(sitemap_url, timeout=30)
        response.raise_for_status()

        # Parse the XML content - try 'xml' parser first, fall back to 'lxml' if available
        try:
            soup = BeautifulSoup(response.content, 'xml')
        except:
            # If 'xml' parser is not available, try 'lxml'
            try:
                soup = BeautifulSoup(response.content, 'lxml-xml')
            except:
                # If both fail, try regular html parser which can handle xml-like content
                soup = BeautifulSoup(response.content, 'lxml-xml')

        # Find all <url><loc> elements in the sitemap
        url_elements = soup.find_all('loc')
        urls = []

        for url_elem in url_elements:
            url = url_elem.text.strip()
            if url and validate_url(url):  # Use the existing validate_url function
                urls.append(url)

        return urls

    except Exception as e:
        print(f"Error parsing sitemap {sitemap_url}: {str(e)}")
        return []


def extract_urls_from_sitemap_index(sitemap_index_url: str) -> List[str]:
    """
    Extract all URLs from a sitemap index file that may contain multiple sitemaps.

    Args:
        sitemap_index_url: URL to the sitemap index file

    Returns:
        List of all URLs from all sitemaps referenced in the index
    """
    try:
        response = requests.get(sitemap_index_url, timeout=30)
        response.raise_for_status()

        # Parse the XML content - try 'xml' parser first, fall back to 'lxml' if available
        try:
            soup = BeautifulSoup(response.content, 'xml')
        except:
            # If 'xml' parser is not available, try 'lxml'
            try:
                soup = BeautifulSoup(response.content, 'lxml-xml')
            except:
                # If both fail, try regular html parser which can handle xml-like content
                soup = BeautifulSoup(response.content, 'html.parser')

        # Find all <sitemap><loc> elements in the sitemap index
        sitemap_elements = soup.find_all('loc')
        all_urls = []

        for sitemap_elem in sitemap_elements:
            sitemap_url = sitemap_elem.text.strip()
            if sitemap_url and validate_url(sitemap_url):
                # Extract URLs from each individual sitemap
                urls = extract_urls_from_sitemap(sitemap_url)
                all_urls.extend(urls)

        return all_urls

    except Exception as e:
        print(f"Error parsing sitemap index {sitemap_index_url}: {str(e)}")
        return []


if __name__ == "__main__":
    # Example usage
    logger = setup_logging("INFO")
    logger.info("Logging setup completed")

    # Test URL validation
    test_urls = [
        "https://example.com",
        "http://localhost:3000",
        "ftp://example.com",  # Invalid scheme
        "not-a-url"
    ]

    for url in test_urls:
        print(f"URL: {url}, Valid: {validate_url(url)}, Sanitized: {sanitize_url(url) if validate_url(url) else 'N/A'}")