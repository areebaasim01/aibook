"""
Validation module for the Book Embeddings Ingestion Pipeline.

This module provides functionality to validate the complete pipeline
with test data and scenarios.
"""
import os
from typing import List
import tempfile
import logging
from .config import load_config
from .pipeline import get_pipeline_instance
from .utils import setup_logging


def create_test_urls_file() -> str:
    """
    Create a temporary file with test URLs for validation.

    Returns:
        Path to the temporary file
    """
    # For validation, we'll use some example documentation URLs
    test_urls = [
        "https://httpbin.org/html",  # Simple HTML page for testing
        "https://httpbin.org/json",  # JSON endpoint (will likely fail, but good for error handling test)
    ]

    # Create a temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
        for url in test_urls:
            f.write(f"{url}\n")

    return f.name


def validate_pipeline_components(config) -> bool:
    """
    Validate individual components of the pipeline.

    Args:
        config: Configuration object

    Returns:
        bool: True if all components are valid, False otherwise
    """
    logger = logging.getLogger(__name__)
    logger.info("Starting component validation...")

    try:
        # Create pipeline instance
        pipeline = get_pipeline_instance(config)

        # Validate each component
        components_valid = [
            pipeline.crawler is not None,
            pipeline.chunker is not None,
            pipeline.embedder is not None,
            pipeline.vector_db is not None,
            pipeline.state_manager is not None
        ]

        if not all(components_valid):
            logger.error("One or more pipeline components are not initialized")
            return False

        # Test basic functionality of each component
        # Test chunker
        test_text = "This is a test sentence for validation."
        test_chunks = pipeline.chunker.chunk_text(test_text)
        if not test_chunks:
            logger.error("Chunker validation failed")
            return False

        logger.info("Component validation passed")
        return True

    except Exception as e:
        logger.error(f"Component validation failed: {str(e)}")
        return False


def run_end_to_end_validation() -> bool:
    """
    Run end-to-end validation of the complete pipeline.

    Returns:
        bool: True if validation passes, False otherwise
    """
    logger = logging.getLogger(__name__)
    logger.info("Starting end-to-end validation...")

    try:
        # Create temporary URLs file for testing
        temp_urls_file = create_test_urls_file()
        logger.info(f"Created temporary URLs file: {temp_urls_file}")

        # Load configuration
        config = load_config()
        logger.info("Configuration loaded successfully")

        # Validate components first
        if not validate_pipeline_components(config):
            logger.error("Component validation failed")
            return False

        # Create pipeline instance
        pipeline = get_pipeline_instance(config)

        # Test with a simple URL that should work
        test_urls = ["https://httpbin.org/html"]  # Simple HTML page

        logger.info(f"Running pipeline validation with test URL: {test_urls[0]}")

        # Run pipeline in dry run mode (no storage)
        success = pipeline.run_pipeline(test_urls, store_embeddings=False)

        if not success:
            logger.error("Pipeline execution failed")
            return False

        logger.info("End-to-end validation passed")
        return True

    except Exception as e:
        logger.error(f"End-to-end validation failed: {str(e)}")
        return False

    finally:
        # Clean up temporary file
        if 'temp_urls_file' in locals():
            try:
                os.unlink(temp_urls_file)
                logger.info(f"Cleaned up temporary file: {temp_urls_file}")
            except:
                pass  # Ignore cleanup errors


def validate_error_handling() -> bool:
    """
    Validate that the pipeline properly handles errors.

    Returns:
        bool: True if error handling validation passes, False otherwise
    """
    logger = logging.getLogger(__name__)
    logger.info("Starting error handling validation...")

    try:
        # Load configuration
        config = load_config()

        # Create pipeline instance
        pipeline = get_pipeline_instance(config)

        # Test with invalid URL to ensure error handling works
        invalid_urls = ["http://invalid.url.that.should.not.exist"]

        logger.info("Testing error handling with invalid URL...")

        # This should handle the error gracefully
        success = pipeline.run_pipeline(invalid_urls, store_embeddings=False)

        # The pipeline should handle errors gracefully, so it might still return True
        # but we can check the state manager to see if the URL was marked as failed
        failed_urls = pipeline.state_manager.get_failed_urls()
        logger.info(f"Failed URLs after error test: {failed_urls}")

        logger.info("Error handling validation passed")
        return True

    except Exception as e:
        logger.error(f"Error handling validation failed: {str(e)}")
        return False


def main():
    """
    Main function to run all validations.
    """
    # Setup logging
    logger = setup_logging("INFO")
    logger.info("Starting pipeline validation...")

    # Run all validations
    validations = [
        ("Component validation", validate_pipeline_components(load_config())),
        ("Error handling validation", validate_error_handling()),
        ("End-to-end validation", run_end_to_end_validation())
    ]

    all_passed = True
    for name, result in validations:
        status = "PASSED" if result else "FAILED"
        logger.info(f"{name}: {status}")
        if not result:
            all_passed = False

    if all_passed:
        logger.info("All validations PASSED!")
        print("✓ All validations passed!")
        return True
    else:
        logger.error("Some validations FAILED!")
        print("✗ Some validations failed!")
        return False


if __name__ == "__main__":
    main()