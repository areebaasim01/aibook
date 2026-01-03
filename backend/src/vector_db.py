"""
Vector database module for the Book Embeddings Ingestion Pipeline.

This module handles interactions with Qdrant vector database for storing
and retrieving embeddings.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from .models import DocumentChunk
import logging


class VectorDB:
    """
    Class to handle all interactions with the Qdrant vector database.
    """

    def __init__(self, config):
        """
        Initialize the VectorDB with configuration.

        Args:
            config: Configuration object with Qdrant settings
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Initialize Qdrant client based on configuration
        if config.qdrant_url:
            # Use URL if provided (for cloud instances)
            self.client = QdrantClient(
                url=config.qdrant_url,
                api_key=config.qdrant_api_key,
                timeout=10
            )
        else:
            # Use host/port for local instances
            self.client = QdrantClient(
                host=config.qdrant_host,
                port=config.qdrant_port,
                timeout=10
            )

        self.collection_name = config.qdrant_collection_name

    def setup_collection(self) -> bool:
        """
        Setup the Qdrant collection with proper configuration.

        Creates the collection if it doesn't exist with 1024-dimensional vectors
        using cosine distance metric as specified in the requirements.

        Returns:
            bool: True if setup was successful, False otherwise
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name in collection_names:
                self.logger.info(f"Collection '{self.collection_name}' already exists")
                return True

            # Create collection with 1024-dimensional vectors and cosine distance
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=1024,  # Cohere embeddings are 1024-dimensional
                    distance=Distance.COSINE
                )
            )

            self.logger.info(f"Created collection '{self.collection_name}' with 1024-dimensional vectors")
            return True

        except Exception as e:
            self.logger.error(f"Failed to setup collection '{self.collection_name}': {str(e)}")
            return False

    def store_embeddings(self, chunks: List[DocumentChunk]) -> bool:
        """
        Store document chunks with their embeddings in Qdrant.

        Args:
            chunks: List of DocumentChunk objects to store

        Returns:
            bool: True if storage was successful, False otherwise
        """
        try:
            # Prepare points for insertion
            points = []
            for chunk in chunks:
                if chunk.embedding is None:
                    self.logger.warning(f"Chunk {chunk.id} has no embedding, skipping")
                    continue

                point = models.PointStruct(
                    id=chunk.id,
                    vector=chunk.embedding,
                    payload=chunk.to_payload()
                )
                points.append(point)

            if not points:
                self.logger.warning("No points to store")
                return True

            # Insert points into collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            self.logger.info(f"Successfully stored {len(points)} embeddings in '{self.collection_name}'")
            return True

        except Exception as e:
            self.logger.error(f"Failed to store embeddings: {str(e)}")
            return False

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in the collection.

        Args:
            query_embedding: The embedding vector to search for similarity
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing similar chunks with their metadata
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )

            # Convert results to a more usable format
            search_results = []
            for result in results:
                search_results.append({
                    'id': result.id,
                    'score': result.score,
                    'payload': result.payload,
                    'content': result.payload.get('content', ''),
                    'source_url': result.payload.get('source_url', ''),
                    'page_title': result.payload.get('page_title', ''),
                    'section': result.payload.get('section', '')
                })

            return search_results

        except Exception as e:
            self.logger.error(f"Search failed: {str(e)}")
            return []

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID.

        Args:
            chunk_id: The ID of the chunk to retrieve

        Returns:
            Dictionary containing the chunk data, or None if not found
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id]
            )

            if records:
                record = records[0]
                return {
                    'id': record.id,
                    'vector': record.vector,
                    'payload': record.payload
                }

            return None
        except Exception as e:
            self.logger.error(f"Failed to retrieve chunk {chunk_id}: {str(e)}")
            return None

    def get_total_count(self) -> int:
        """
        Get the total number of vectors stored in the collection.

        Returns:
            int: Total count of vectors in the collection
        """
        try:
            count = self.client.count(
                collection_name=self.collection_name
            )
            return count.count
        except Exception as e:
            self.logger.error(f"Failed to get total count: {str(e)}")
            return 0

    def delete_collection(self) -> bool:
        """
        Delete the entire collection (use with caution!).

        Returns:
            bool: True if deletion was successful, False otherwise
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            self.logger.info(f"Collection '{self.collection_name}' deleted")
            return True
        except Exception as e:
            self.logger.error(f"Failed to delete collection: {str(e)}")
            return False

    def validate_connection(self) -> bool:
        """
        Validate the connection to the Qdrant instance.

        Returns:
            bool: True if connection is valid, False otherwise
        """
        try:
            # Try to get collections list to validate connection
            self.client.get_collections()
            return True
        except Exception as e:
            self.logger.error(f"Qdrant connection validation failed: {str(e)}")
            return False


def get_vector_db_instance(config) -> VectorDB:
    """
    Factory function to create a VectorDB instance.

    Args:
        config: Configuration object with Qdrant settings

    Returns:
        VectorDB: Configured VectorDB instance
    """
    return VectorDB(config)


if __name__ == "__main__":
    # Example usage would require actual config
    print("VectorDB module loaded successfully")
    print("Use get_vector_db_instance(config) to create an instance with your configuration")