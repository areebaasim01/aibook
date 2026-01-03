"""
Main entry point for the Book Embeddings Ingestion Pipeline.

This module provides the command-line interface for the ingestion pipeline.
"""
import argparse
import os
import sys
import json
from typing import List
import logging
from .config import load_config
from .crawler import get_crawler_instance
from .pipeline import get_pipeline_instance
from .state_manager import get_state_manager_instance
from .utils import setup_logging


def load_urls_from_file(file_path: str) -> List[str]:
    """
    Load URLs from a text file, one URL per line.

    Args:
        file_path: Path to the file containing URLs

    Returns:
        List of URLs
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"URL file not found: {file_path}")

    with open(file_path, 'r', encoding='utf-8') as f:
        urls = [line.strip() for line in f if line.strip() and not line.startswith('#')]

    return urls


def load_urls_from_env() -> List[str]:
    """
    Load URLs from the COHERE_URLS environment variable.

    Returns:
        List of URLs
    """
    urls_env = os.getenv('COHERE_URLS')
    if not urls_env:
        return []

    try:
        urls = json.loads(urls_env)
        if not isinstance(urls, list):
            raise ValueError("COHERE_URLS environment variable must contain a JSON array of URLs")
        return urls
    except json.JSONDecodeError:
        raise ValueError("COHERE_URLS environment variable must contain valid JSON")


def load_urls_from_command_line(url: str) -> List[str]:
    """
    Load URLs from command line argument.

    Args:
        url: Single URL from command line

    Returns:
        List containing the single URL
    """
    if url:
        return [url]
    return []


def get_all_urls(url_file: str = None, env_var: bool = False, command_url: str = None) -> List[str]:
    """
    Get URLs from various sources with priority: command line > file > environment variable.

    Args:
        url_file: Path to file containing URLs
        env_var: Whether to check environment variable
        command_url: Single URL from command line

    Returns:
        List of URLs
    """
    # Priority 1: Command line URL
    if command_url:
        return [command_url]

    # Priority 2: File
    if url_file:
        return load_urls_from_file(url_file)

    # Priority 3: Environment variable
    if env_var:
        return load_urls_from_env()

    # Default: Try default file
    default_file = "urls.txt"
    if os.path.exists(default_file):
        print(f"Using default URL file: {default_file}")
        return load_urls_from_file(default_file)

    # No URLs found
    return []


def main():
    """
    Main function to run the ingestion pipeline.
    """
    parser = argparse.ArgumentParser(description="Book Embeddings Ingestion Pipeline")
    parser.add_argument("--url", type=str, help="Single URL to process")
    parser.add_argument("--url-file", type=str, default=None, help="Path to file containing URLs (one per line)")
    parser.add_argument("--use-env", action="store_true", help="Use URLs from COHERE_URLS environment variable")
    parser.add_argument("--no-store", action="store_true", help="Don't store embeddings in vector database (dry run)")
    parser.add_argument("--resume", action="store_true", help="Resume processing from last saved state")
    parser.add_argument("--log-level", type=str, default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                        help="Logging level")

    args = parser.parse_args()

    # Setup logging
    logger = setup_logging(args.log_level)
    logger.info("Starting Book Embeddings Ingestion Pipeline")

    try:
        # Load configuration
        config = load_config()
        logger.info("Configuration loaded successfully")

        # Get URLs to process
        urls = get_all_urls(args.url_file, args.use_env, args.url)

        if not urls:
            print("No URLs provided. Please specify URLs via:")
            print("  - Command line argument: --url <url>")
            print("  - File: --url-file <file_path>")
            print("  - Environment variable: export COHERE_URLS='[\"url1\", \"url2\", ...]'")
            print("  - Default file: urls.txt (in current directory)")
            sys.exit(1)

        logger.info(f"Processing {len(urls)} URLs: {urls[:5]}{'...' if len(urls) > 5 else ''}")

        # Create pipeline instance
        pipeline = get_pipeline_instance(config)

        # If resume is requested, use remaining URLs based on state
        if args.resume:
            logger.info("Resuming from saved state...")
            remaining_urls = pipeline.state_manager.get_remaining_urls(urls)
            logger.info(f"Found {len(remaining_urls)} URLs remaining to process")
            urls = remaining_urls

            if not urls:
                logger.info("All URLs have already been processed successfully!")
                print("All URLs have already been processed successfully!")
                sys.exit(0)

        # Validate pipeline components
        if not pipeline.validate_pipeline():
            logger.error("Pipeline validation failed")
            sys.exit(1)

        logger.info("Pipeline validation passed")

        # Run the ingestion pipeline
        success = pipeline.run_pipeline(urls, store_embeddings=not args.no_store)

        if success:
            logger.info("Ingestion pipeline completed successfully!")
            print(f"Successfully processed {len(urls)} URLs")
        else:
            logger.error("Ingestion pipeline failed")
            sys.exit(1)

    except Exception as e:
        logger.error(f"Pipeline failed with error: {str(e)}")
        sys.exit(1)


def run_crawler_only(urls: List[str], config):
    """
    Run just the crawler component for testing purposes.

    Args:
        urls: List of URLs to crawl
        config: Configuration object
    """
    logger = logging.getLogger(__name__)
    crawler = get_crawler_instance(config)

    logger.info(f"Running crawler only on {len(urls)} URLs")
    results = crawler.process_urls(urls)

    for result in results:
        if result.status == "completed":
            logger.info(f"✓ Successfully crawled {result.url} ({result.word_count} words)")
        else:
            logger.error(f"✗ Failed to crawl {result.url}: {result.error_message}")

    return results


if __name__ == "__main__":
    main()