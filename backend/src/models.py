"""
Data models for the Book Embeddings Ingestion Pipeline.

This module defines the data structures used throughout the application,
including Document Chunk and Processing State entities.
"""
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any
from datetime import datetime
from enum import Enum
import uuid


class ProcessingStatus(Enum):
    """Enumeration of possible processing statuses."""
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class DocumentChunk:
    """
    Represents a segment of text extracted from a web page that has been processed into an embedding.
    """
    # Unique identifier for the chunk
    id: str = field(default_factory=lambda: str(uuid.uuid4()))

    # The text content of the chunk
    content: str = ""

    # URL where the content was found
    source_url: str = ""

    # Title of the source page
    page_title: Optional[str] = None

    # Section/heading where the content appears
    section: Optional[str] = None

    # Position of the chunk within the original document
    position: int = 0

    # Vector representation of the content (1024-dimensional for Cohere)
    embedding: Optional[List[float]] = None

    # Timestamp of when the chunk was processed
    created_at: datetime = field(default_factory=datetime.utcnow)

    # Additional metadata about the chunk
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_payload(self) -> Dict[str, Any]:
        """
        Convert the DocumentChunk to a payload suitable for storage in Qdrant.

        Returns:
            Dict containing the chunk data in Qdrant-compatible format
        """
        return {
            "content": self.content,
            "source_url": self.source_url,
            "page_title": self.page_title or "",
            "section": self.section or "",
            "position": self.position,
            "created_at": self.created_at.isoformat(),
            "metadata": self.metadata
        }

    @classmethod
    def from_payload(cls, payload: Dict[str, Any]) -> 'DocumentChunk':
        """
        Create a DocumentChunk from a Qdrant payload.

        Args:
            payload: Dictionary containing chunk data from Qdrant

        Returns:
            DocumentChunk instance
        """
        return cls(
            id=payload.get("id", str(uuid.uuid4())),
            content=payload.get("content", ""),
            source_url=payload.get("source_url", ""),
            page_title=payload.get("page_title"),
            section=payload.get("section"),
            position=payload.get("position", 0),
            created_at=datetime.fromisoformat(payload.get("created_at")) if payload.get("created_at") else datetime.utcnow(),
            metadata=payload.get("metadata", {})
        )


@dataclass
class ProcessingState:
    """
    Tracks the processing status of URLs to enable resume functionality.
    """
    # The URL being processed
    url: str

    # Current status of processing
    status: ProcessingStatus = ProcessingStatus.PENDING

    # Timestamp of last processing attempt
    last_processed: Optional[datetime] = None

    # Number of retry attempts
    retry_count: int = 0

    # Any error encountered during processing
    error_message: Optional[str] = None

    def mark_processing(self):
        """Mark the URL as currently being processed."""
        self.status = ProcessingStatus.PROCESSING
        self.last_processed = datetime.utcnow()

    def mark_completed(self):
        """Mark the URL as successfully processed."""
        self.status = ProcessingStatus.COMPLETED
        self.last_processed = datetime.utcnow()
        self.retry_count = 0
        self.error_message = None

    def mark_failed(self, error_message: Optional[str] = None):
        """Mark the URL as failed processing."""
        self.status = ProcessingStatus.FAILED
        self.last_processed = datetime.utcnow()
        self.retry_count += 1
        self.error_message = error_message

    def reset(self):
        """Reset the processing state to pending."""
        self.status = ProcessingStatus.PENDING
        self.retry_count = 0
        self.error_message = None
        self.last_processed = None


@dataclass
class ProcessedURL:
    """
    Represents a processed URL with its chunks and processing results.
    """
    url: str
    chunks: List[DocumentChunk]
    processing_time: float  # in seconds
    status: ProcessingStatus
    error_message: Optional[str] = None
    page_title: Optional[str] = None
    word_count: int = 0
    chunk_count: int = 0


@dataclass
class EmbeddingResult:
    """
    Represents the result of an embedding operation.
    """
    chunk_id: str
    text: str
    embedding: List[float]
    processing_time: float  # in seconds
    success: bool = True
    error_message: Optional[str] = None


@dataclass
class StorageResult:
    """
    Represents the result of a storage operation in Qdrant.
    """
    chunk_ids: List[str]
    stored_count: int
    processing_time: float  # in seconds
    success: bool = True
    error_message: Optional[str] = None


if __name__ == "__main__":
    # Example usage
    chunk = DocumentChunk(
        content="This is a sample text chunk for embedding.",
        source_url="https://example.com/page",
        page_title="Example Page",
        section="Introduction",
        position=0
    )

    print(f"Created chunk with ID: {chunk.id}")
    print(f"Chunk content: {chunk.content[:50]}...")
    print(f"Source URL: {chunk.source_url}")
    print(f"Payload for Qdrant: {chunk.to_payload()}")

    # Example processing state
    state = ProcessingState(url="https://example.com/page")
    print(f"Initial state for {state.url}: {state.status.value}")

    state.mark_processing()
    print(f"After marking as processing: {state.status.value}")