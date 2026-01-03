"""
Configuration management module for the Book Embeddings Ingestion Pipeline.

This module handles loading and validating configuration from environment variables
and configuration files.
"""
import os
from dataclasses import dataclass
from typing import Optional
from dotenv import load_dotenv


@dataclass
class Config:
    """Configuration class holding all application settings."""

    # Cohere settings
    cohere_api_key: str
    cohere_model: str = "embed-english-v3.0"

    # Qdrant settings
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_host: str = "localhost"
    qdrant_port: int = 6333
    qdrant_collection_name: str = "book_embeddings"

    # Processing settings
    chunk_size: int = 512
    max_concurrent_requests: int = 5
    retry_attempts: int = 3
    batch_size: int = 10  # For Cohere API batch processing


def load_config() -> Config:
    """
    Load configuration from environment variables and .env file.

    Returns:
        Config: A configuration object with all settings loaded and validated.
    """
    # Load from .env file if it exists
    load_dotenv(".env")

    # Load Cohere settings
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable is required")

    # Load Qdrant settings
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_host = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port = int(os.getenv("QDRANT_PORT", "6333"))
    qdrant_collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

    # Load processing settings
    chunk_size = int(os.getenv("CHUNK_SIZE", "512"))
    cohere_model = os.getenv("COHERE_MODEL", "embed-english-v3.0")

    return Config(
        cohere_api_key=cohere_api_key,
        cohere_model=cohere_model,
        qdrant_url=qdrant_url,
        qdrant_api_key=qdrant_api_key,
        qdrant_host=qdrant_host,
        qdrant_port=qdrant_port,
        qdrant_collection_name=qdrant_collection_name,
        chunk_size=chunk_size,
    )


if __name__ == "__main__":
    # Example usage
    config = load_config()
    print(f"Configuration loaded successfully:")
    print(f"- Cohere model: {config.cohere_model}")
    print(f"- Qdrant collection: {config.qdrant_collection_name}")
    print(f"- Chunk size: {config.chunk_size}")