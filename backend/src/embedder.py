"""
Embedding generation module for the Book Embeddings Ingestion Pipeline.

This module handles generating embeddings using the Cohere API.
"""
import cohere
from typing import List, Dict, Optional
import time
import logging
from .models import DocumentChunk, EmbeddingResult
from .utils import retry_with_backoff


class Embedder:
    """
    Class to handle embedding generation using Cohere API.
    """

    def __init__(self, config):
        """
        Initialize the Embedder with configuration.

        Args:
            config: Configuration object with Cohere settings
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Initialize Cohere client
        self.client = cohere.Client(config.cohere_api_key)

        # Simple cache to avoid redundant API calls
        self.cache = {}

    def generate_embeddings(self, texts: List[str]) -> List[EmbeddingResult]:
        """
        Generate embeddings for a list of texts using Cohere API.

        Args:
            texts: List of text strings to embed

        Returns:
            List of EmbeddingResult objects
        """
        results = []

        # Filter out empty texts
        valid_texts = [text for text in texts if text and len(text.strip()) > 0]

        if not valid_texts:
            self.logger.warning("No valid texts provided for embedding")
            return []

        try:
            # Use Cohere's embed API
            response = self.client.embed(
                texts=valid_texts,
                model=self.config.cohere_model,
                input_type="search_document"  # Optimal for document search
            )

            embeddings = response.embeddings

            # Create embedding results
            for i, text in enumerate(valid_texts):
                if i < len(embeddings):
                    result = EmbeddingResult(
                        chunk_id=f"chunk_{i}",  # This will be replaced with actual chunk ID in full pipeline
                        text=text,
                        embedding=embeddings[i],
                        processing_time=0,  # Will be calculated in the actual pipeline
                        success=True
                    )
                    results.append(result)
                else:
                    result = EmbeddingResult(
                        chunk_id=f"chunk_{i}",
                        text=text,
                        embedding=[],
                        processing_time=0,
                        success=False,
                        error_message="No embedding returned from API"
                    )
                    results.append(result)

            self.logger.info(f"Generated embeddings for {len(valid_texts)} texts")
            return results

        except Exception as e:
            self.logger.error(f"Error generating embeddings: {str(e)}")
            # Return error results for all texts
            error_results = []
            for i, text in enumerate(valid_texts):
                error_results.append(EmbeddingResult(
                    chunk_id=f"chunk_{i}",
                    text=text,
                    embedding=[],
                    processing_time=0,
                    success=False,
                    error_message=str(e)
                ))
            return error_results

    def generate_embeddings_for_chunks(self, chunks: List[DocumentChunk]) -> List[DocumentChunk]:
        """
        Generate embeddings for a list of DocumentChunk objects.

        Args:
            chunks: List of DocumentChunk objects to embed

        Returns:
            List of DocumentChunk objects with embeddings added
        """
        if not chunks:
            return []

        # Extract text content from chunks
        texts = [chunk.content for chunk in chunks if chunk.content and len(chunk.content.strip()) > 0]

        if not texts:
            self.logger.warning("No valid text content found in chunks for embedding")
            return chunks

        # Generate embeddings
        start_time = time.time()
        embedding_results = self.generate_embeddings(texts)
        processing_time = time.time() - start_time

        # Update chunks with embeddings
        updated_chunks = []
        result_idx = 0

        for chunk in chunks:
            if chunk.content and len(chunk.content.strip()) > 0:
                if result_idx < len(embedding_results) and embedding_results[result_idx].success:
                    # Create a new chunk with the embedding
                    updated_chunk = DocumentChunk(
                        id=chunk.id,
                        content=chunk.content,
                        source_url=chunk.source_url,
                        page_title=chunk.page_title,
                        section=chunk.section,
                        position=chunk.position,
                        embedding=embedding_results[result_idx].embedding,
                        created_at=chunk.created_at,
                        metadata=chunk.metadata
                    )
                    updated_chunk.metadata["embedding_time"] = processing_time / len(texts)  # Average time per chunk
                    updated_chunks.append(updated_chunk)
                else:
                    # Add the original chunk without embedding but mark the error
                    updated_chunk = chunk
                    error_msg = embedding_results[result_idx].error_message if result_idx < len(embedding_results) else "Unknown error"
                    updated_chunk.metadata["embedding_error"] = error_msg
                    updated_chunks.append(updated_chunk)

                result_idx += 1
            else:
                # Add chunks with empty content as-is
                updated_chunks.append(chunk)

        self.logger.info(f"Added embeddings to {len([c for c in updated_chunks if c.embedding])} out of {len(updated_chunks)} chunks")
        return updated_chunks

    def generate_embeddings_batch(self, texts: List[str], batch_size: int = 96) -> List[EmbeddingResult]:
        """
        Generate embeddings in batches to handle large lists of texts efficiently.

        Args:
            texts: List of text strings to embed
            batch_size: Size of each batch (Cohere API has limits)

        Returns:
            List of EmbeddingResult objects
        """
        all_results = []

        # Split texts into batches
        batches = [texts[i:i + batch_size] for i in range(0, len(texts), batch_size)]

        for i, batch in enumerate(batches):
            self.logger.info(f"Processing batch {i+1}/{len(batches)} with {len(batch)} texts")

            batch_results = self.generate_embeddings(batch)
            all_results.extend(batch_results)

            # Small delay between batches to be respectful to the API
            if i < len(batches) - 1:  # Don't delay after the last batch
                time.sleep(0.1)

        return all_results

    def cache_embeddings(self, text: str, embedding: List[float]):
        """
        Cache an embedding for a specific text to avoid redundant API calls.

        Args:
            text: The text that was embedded
            embedding: The generated embedding vector
        """
        # Use a simple hash of the text as the cache key
        cache_key = hash(text)
        self.cache[cache_key] = embedding

    def get_cached_embedding(self, text: str) -> Optional[List[float]]:
        """
        Get a cached embedding for a text if it exists.

        Args:
            text: The text to look up

        Returns:
            Cached embedding if found, None otherwise
        """
        cache_key = hash(text)
        return self.cache.get(cache_key)

    def embed_chunks_with_caching(self, chunks: List[DocumentChunk]) -> List[DocumentChunk]:
        """
        Generate embeddings for chunks with caching to avoid redundant API calls.

        Args:
            chunks: List of DocumentChunk objects to embed

        Returns:
            List of DocumentChunk objects with embeddings added
        """
        uncached_chunks = []
        cached_results = []

        # Check which chunks have cached embeddings
        for i, chunk in enumerate(chunks):
            cached_embedding = self.get_cached_embedding(chunk.content)
            if cached_embedding:
                # Create a new chunk with the cached embedding
                updated_chunk = DocumentChunk(
                    id=chunk.id,
                    content=chunk.content,
                    source_url=chunk.source_url,
                    page_title=chunk.page_title,
                    section=chunk.section,
                    position=chunk.position,
                    embedding=cached_embedding,
                    created_at=chunk.created_at,
                    metadata=chunk.metadata
                )
                cached_results.append(updated_chunk)
            else:
                uncached_chunks.append(chunk)

        # Generate embeddings for uncached chunks
        updated_uncached = self.generate_embeddings_for_chunks(uncached_chunks)

        # Combine cached and newly generated results
        all_results = cached_results + updated_uncached

        # Sort by original position to maintain order
        all_results.sort(key=lambda x: x.position)

        self.logger.info(f"Used cached embeddings for {len(cached_results)}, generated {len(updated_uncached)} new embeddings")
        return all_results


def get_embedder_instance(config) -> Embedder:
    """
    Factory function to create an Embedder instance.

    Args:
        config: Configuration object

    Returns:
        Embedder: Configured Embedder instance
    """
    return Embedder(config)


if __name__ == "__main__":
    # Example usage would require actual config with Cohere API key
    print("Embedder module loaded successfully")
    print("Use get_embedder_instance(config) to create an instance with your configuration")