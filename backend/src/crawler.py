"""
URL crawling and content extraction module for the Book Embeddings Ingestion Pipeline.

This module handles fetching URLs, extracting clean text content from web pages,
and handling various content types and error scenarios.
"""
import requests
from bs4 import BeautifulSoup
from typing import List, Dict, Optional, Tuple
import time
import logging
from urllib.parse import urljoin, urlparse
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
from .utils import sanitize_url, validate_url, is_valid_docusaurus_url, retry_with_backoff
from .models import ProcessedURL, ProcessingStatus
import re


class Crawler:
    """
    Class to handle URL crawling and content extraction.
    """

    def __init__(self, config):
        """
        Initialize the Crawler with configuration.

        Args:
            config: Configuration object with settings
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Setup session with retry strategy
        self.session = requests.Session()

        # Define retry strategy
        retry_strategy = Retry(
            total=config.retry_attempts,
            backoff_factor=1,
            status_forcelist=[429, 500, 502, 503, 504],
        )

        adapter = HTTPAdapter(max_retries=retry_strategy)
        self.session.mount("http://", adapter)
        self.session.mount("https://", adapter)

        # Set headers to mimic a real browser
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })

    def fetch_url(self, url: str) -> Tuple[Optional[str], Optional[str]]:
        """
        Fetch content from a URL and return the raw HTML/text.

        Args:
            url: The URL to fetch

        Returns:
            Tuple of (content, error_message) - content if successful, None if failed
        """
        try:
            # Validate and sanitize URL
            if not validate_url(url):
                return None, f"Invalid URL format: {url}"

            sanitized_url = sanitize_url(url)

            # Make the request
            response = self.session.get(sanitized_url, timeout=30)
            response.raise_for_status()  # Raise an exception for bad status codes

            # Check content type
            content_type = response.headers.get('content-type', '').lower()
            if 'text/html' not in content_type and 'application/xhtml+xml' not in content_type:
                self.logger.warning(f"URL {url} returned non-HTML content type: {content_type}")

            return response.text, None

        except requests.exceptions.RequestException as e:
            error_msg = f"Failed to fetch {url}: {str(e)}"
            self.logger.error(error_msg)
            return None, error_msg
        except Exception as e:
            error_msg = f"Unexpected error fetching {url}: {str(e)}"
            self.logger.error(error_msg)
            return None, error_msg

    def extract_content_from_html(self, html_content: str, url: str = "") -> Tuple[str, str]:
        """
        Extract clean text content from HTML, removing navigation and other irrelevant elements.

        Args:
            html_content: Raw HTML content
            url: The source URL (for context)

        Returns:
            Tuple of (clean_text, page_title)
        """
        try:
            soup = BeautifulSoup(html_content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                script.decompose()

            # Try to extract the main content area - common patterns for documentation sites
            main_content = None

            # Look for common content containers in documentation sites
            for selector in ['main', '.main-content', '.content', '.doc-content',
                           '.docs-content', '.markdown', '.container', '.wrapper']:
                main_content = soup.select_one(selector)
                if main_content:
                    break

            # If no specific main content found, use the body
            if not main_content:
                main_content = soup.find('main') or soup.find('article') or soup.find('body') or soup

            # Remove common navigation and sidebar elements
            for element in main_content.find_all(['nav', 'aside', 'header', 'footer']):
                element.decompose()

            # Remove elements with common class names for navigation
            for element in main_content.find_all(class_=re.compile(r'nav|sidebar|menu|toc|header|footer')):
                element.decompose()

            # Extract the page title
            title_tag = soup.find('title')
            page_title = title_tag.get_text().strip() if title_tag else "No Title"

            # Get the cleaned text
            clean_text = main_content.get_text(separator=' ', strip=True)

            # Clean up extra whitespace
            clean_text = re.sub(r'\s+', ' ', clean_text).strip()

            # Remove common documentation site elements (breadcrumbs, etc.)
            # Remove lines that look like navigation or breadcrumbs
            lines = clean_text.split('\n')
            filtered_lines = []
            for line in lines:
                line = line.strip()
                # Skip lines that look like navigation or breadcrumbs
                if len(line) > 1 and not re.match(r'^[\w\s/\\-]*$', line) or len(line) > 3:
                    filtered_lines.append(line)

            clean_text = ' '.join(filtered_lines)

            return clean_text, page_title

        except Exception as e:
            self.logger.error(f"Error extracting content from {url}: {str(e)}")
            # Return basic text extraction as fallback
            soup = BeautifulSoup(html_content, 'html.parser')
            clean_text = soup.get_text(separator=' ', strip=True)
            clean_text = re.sub(r'\s+', ' ', clean_text).strip()
            title_tag = soup.find('title')
            page_title = title_tag.get_text().strip() if title_tag else "No Title"
            return clean_text, page_title

    def process_url(self, url: str) -> ProcessedURL:
        """
        Process a single URL by fetching and extracting content.

        Args:
            url: The URL to process

        Returns:
            ProcessedURL object with the results
        """
        start_time = time.time()

        try:
            # Fetch the content
            html_content, error = self.fetch_url(url)
            if error:
                return ProcessedURL(
                    url=url,
                    chunks=[],
                    processing_time=time.time() - start_time,
                    status=ProcessingStatus.FAILED,
                    error_message=error
                )

            # Extract clean content
            clean_content, page_title = self.extract_content_from_html(html_content, url)

            # Basic validation - check if content is substantial
            if not clean_content or len(clean_content.strip()) < 50:
                return ProcessedURL(
                    url=url,
                    chunks=[],
                    processing_time=time.time() - start_time,
                    status=ProcessingStatus.FAILED,
                    error_message="Insufficient content extracted",
                    page_title=page_title
                )

            self.logger.info(f"Successfully processed {url} - extracted {len(clean_content)} characters")

            return ProcessedURL(
                url=url,
                chunks=[],  # Will be populated in the next step
                processing_time=time.time() - start_time,
                status=ProcessingStatus.COMPLETED,
                page_title=page_title,
                word_count=len(clean_content.split()),
                chunk_count=0
            )

        except Exception as e:
            processing_time = time.time() - start_time
            error_msg = f"Error processing {url}: {str(e)}"
            self.logger.error(error_msg)
            return ProcessedURL(
                url=url,
                chunks=[],
                processing_time=processing_time,
                status=ProcessingStatus.FAILED,
                error_message=error_msg
            )

    def process_urls(self, urls: List[str], max_concurrent: Optional[int] = None) -> List[ProcessedURL]:
        """
        Process multiple URLs with optional concurrency control.

        Args:
            urls: List of URLs to process
            max_concurrent: Maximum number of concurrent requests (defaults to config)

        Returns:
            List of ProcessedURL objects with the results
        """
        if max_concurrent is None:
            max_concurrent = self.config.max_concurrent_requests

        results = []

        # Process URLs sequentially for now (can be enhanced with threading later)
        for i, url in enumerate(urls):
            self.logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")
            result = self.process_url(url)
            results.append(result)

            # Small delay to be respectful to servers
            time.sleep(0.1)

        return results

    def validate_docusaurus_content(self, html_content: str) -> bool:
        """
        Validate if the content appears to be from a Docusaurus site.

        Args:
            html_content: HTML content to validate

        Returns:
            bool: True if content appears to be from Docusaurus, False otherwise
        """
        # Look for common Docusaurus indicators in the HTML
        docusaurus_indicators = [
            'docusaurus',
            'doc-sidebar',
            'doc-page',
            'docs-doc-page',
            'navbar',
            'main-wrapper'
        ]

        content_lower = html_content.lower()
        matches = sum(1 for indicator in docusaurus_indicators if indicator in content_lower)

        # If we find at least 2 indicators, consider it likely a Docusaurus site
        return matches >= 2


def get_crawler_instance(config) -> Crawler:
    """
    Factory function to create a Crawler instance.

    Args:
        config: Configuration object

    Returns:
        Crawler: Configured Crawler instance
    """
    return Crawler(config)


if __name__ == "__main__":
    # Example usage would require actual config
    print("Crawler module loaded successfully")
    print("Use get_crawler_instance(config) to create an instance with your configuration")