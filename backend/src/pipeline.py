"""
Main pipeline module for the Book Embeddings Ingestion Pipeline.

This module orchestrates the complete workflow: crawling, chunking, embedding, and storing.
"""
from typing import List, Optional
import time
import logging
from .models import DocumentChunk, ProcessedURL, ProcessingStatus
from .crawler import Crawler
from .chunker import TextChunker
from .embedder import Embedder
from .vector_db import VectorDB
from .state_manager import StateManager
from .utils import create_progress_bar, setup_logging


class IngestionPipeline:
    """
    Class to orchestrate the complete ingestion pipeline.
    """

    def __init__(self, config):
        """
        Initialize the ingestion pipeline with all required components.

        Args:
            config: Configuration object with all settings
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Initialize all components
        self.crawler = Crawler(config)
        self.chunker = TextChunker(config)
        self.embedder = Embedder(config)
        self.vector_db = VectorDB(config)
        self.state_manager = StateManager(config)

    def process_single_url(self, url: str) -> List[DocumentChunk]:
        """
        Process a single URL through the complete pipeline: crawl -> chunk -> embed.

        Args:
            url: The URL to process

        Returns:
            List of DocumentChunk objects with embeddings
        """
        start_time = time.time()

        # Check current state
        current_state = self.state_manager.get_state(url)
        if current_state and current_state.status == ProcessingStatus.COMPLETED:
            self.logger.info(f"URL {url} already completed, skipping")
            return []  # Or return previously processed chunks if available

        # Mark as processing
        self.state_manager.mark_processing(url)

        try:
            # Step 1: Crawl and extract content
            self.logger.info(f"Starting processing for URL: {url}")
            processed_url = self.crawler.process_url(url)

            if processed_url.status != ProcessingStatus.COMPLETED:
                self.logger.error(f"Failed to crawl {url}: {processed_url.error_message}")
                self.state_manager.mark_failed(url, processed_url.error_message)
                return []

            # Step 2: Extract content from the crawled result and chunk it
            html_content, error = self.crawler.fetch_url(url)
            if error or not html_content:
                self.logger.error(f"Failed to fetch content for chunking: {error or 'No content returned'}")
                self.state_manager.mark_failed(url, error or 'No content returned')
                return []

            clean_content, page_title = self.crawler.extract_content_from_html(html_content, url)
            self.logger.info(f"Chunking content from {url}")
            chunks = self.chunker.chunk_text(clean_content, source_url=url, page_title=page_title or processed_url.page_title)

            # Step 3: Generate embeddings
            self.logger.info(f"Generating embeddings for {len(chunks)} chunks from {url}")
            chunks_with_embeddings = self.embedder.embed_chunks_with_caching(chunks)

            processing_time = time.time() - start_time
            self.logger.info(f"Completed processing {url} in {processing_time:.2f} seconds, created {len(chunks_with_embeddings)} chunks with embeddings")

            # Mark as completed
            self.state_manager.mark_completed(url)

            return chunks_with_embeddings

        except Exception as e:
            error_msg = f"Error processing {url}: {str(e)}"
            self.logger.error(error_msg)
            self.state_manager.mark_failed(url, error_msg)
            return []

    def process_multiple_urls(self, urls: List[str]) -> List[DocumentChunk]:
        """
        Process multiple URLs through the complete pipeline.

        Args:
            urls: List of URLs to process

        Returns:
            List of all DocumentChunk objects with embeddings from all URLs
        """
        # Get URLs that still need processing
        remaining_urls = self.state_manager.get_remaining_urls(urls)
        total_remaining = len(remaining_urls)
        total_original = len(urls)

        self.logger.info(f"Starting processing of {total_original} URLs, {total_remaining} remaining to be processed")

        all_chunks = []

        if total_remaining == 0:
            self.logger.info("All URLs have already been processed successfully")
            # Optionally return already processed chunks if stored somewhere
            return []

        # Create progress bar
        progress_bar = create_progress_bar(total_remaining, "Processing URLs")

        for i, url in enumerate(remaining_urls):
            self.logger.info(f"Processing URL {i+1}/{total_remaining}: {url}")

            chunks = self.process_single_url(url)
            all_chunks.extend(chunks)

            progress_bar.update(1)

        progress_bar.close()

        self.logger.info(f"Completed processing {total_remaining}/{total_original} URLs, created {len(all_chunks)} total chunks with embeddings")
        self.state_manager.print_statistics()

        return all_chunks

    def run_pipeline(self, urls: List[str], store_embeddings: bool = True) -> bool:
        """
        Run the complete ingestion pipeline: crawl -> chunk -> embed -> store.

        Args:
            urls: List of URLs to process
            store_embeddings: Whether to store embeddings in vector database

        Returns:
            bool: True if pipeline completed successfully, False otherwise
        """
        start_time = time.time()
        self.logger.info(f"Starting ingestion pipeline for {len(urls)} URLs")

        try:
            # Setup the collection if storing embeddings
            if store_embeddings:
                if not self.vector_db.setup_collection():
                    self.logger.error("Failed to setup vector database collection")
                    return False

            # Process all URLs
            all_chunks = self.process_multiple_urls(urls)

            if not all_chunks:
                self.logger.warning("No chunks were created from the provided URLs")
                return False

            # Store embeddings if requested
            if store_embeddings:
                self.logger.info(f"Storing {len(all_chunks)} embeddings in vector database")
                success = self.vector_db.store_embeddings(all_chunks)
                if not success:
                    self.logger.error("Failed to store embeddings in vector database")
                    return False

            total_time = time.time() - start_time
            self.logger.info(f"Ingestion pipeline completed successfully in {total_time:.2f} seconds")
            self.logger.info(f"Processed {len(urls)} URLs, created {len(all_chunks)} chunks with embeddings")

            return True

        except Exception as e:
            self.logger.error(f"Pipeline failed with error: {str(e)}")
            return False

    def validate_pipeline(self) -> bool:
        """
        Validate that all pipeline components are properly initialized and connected.

        Returns:
            bool: True if all components are valid, False otherwise
        """
        try:
            # Validate each component
            components_valid = [
                self.crawler is not None,
                self.chunker is not None,
                self.embedder is not None,
                self.vector_db is not None
            ]

            if not all(components_valid):
                self.logger.error("One or more pipeline components are not initialized")
                return False

            # Test basic functionality of each component
            # Test crawler
            # (Skipping actual URL test to avoid network calls in validation)

            # Test chunker
            test_text = "This is a test sentence for chunking validation."
            test_chunks = self.chunker.chunk_text(test_text)
            if not test_chunks:
                self.logger.error("Chunker validation failed")
                return False

            # Test embedder (this would require a valid API key in real usage)
            # For now, just check if the client is initialized
            if self.embedder.client is None:
                self.logger.error("Embedder client not initialized")
                return False

            # Test vector DB connection
            if not self.vector_db.validate_connection():
                self.logger.error("Vector database connection validation failed")
                return False

            self.logger.info("Pipeline validation successful")
            return True

        except Exception as e:
            self.logger.error(f"Pipeline validation failed: {str(e)}")
            return False


def get_pipeline_instance(config) -> IngestionPipeline:
    """
    Factory function to create an IngestionPipeline instance.

    Args:
        config: Configuration object

    Returns:
        IngestionPipeline: Configured pipeline instance
    """
    return IngestionPipeline(config)


if __name__ == "__main__":
    # Example usage would require actual config
    print("Pipeline module loaded successfully")
    print("Use get_pipeline_instance(config) to create an instance with your configuration")