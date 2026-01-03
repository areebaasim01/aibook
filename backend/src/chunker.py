"""
Text chunking module for the Book Embeddings Ingestion Pipeline.

This module handles splitting text content into appropriately sized chunks
for embedding generation.
"""
from typing import List, Tuple
from .models import DocumentChunk
import re
import logging


class TextChunker:
    """
    Class to handle text chunking with configurable chunk size.
    """

    def __init__(self, config):
        """
        Initialize the TextChunker with configuration.

        Args:
            config: Configuration object with chunking settings
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.chunk_size = config.chunk_size  # This is in tokens/characters, not necessarily words

    def chunk_text(self, text: str, source_url: str = "", page_title: str = "") -> List[DocumentChunk]:
        """
        Split text into chunks of appropriate size for embedding.

        Args:
            text: The text to chunk
            source_url: URL where the text originated (for metadata)
            page_title: Title of the source page (for metadata)

        Returns:
            List of DocumentChunk objects
        """
        if not text or len(text.strip()) == 0:
            return []

        # Split text into sentences first to avoid breaking sentences across chunks
        sentences = self._split_into_sentences(text)

        chunks = []
        current_chunk = ""
        chunk_position = 0

        for sentence in sentences:
            # Check if adding this sentence would exceed the chunk size
            if len(current_chunk) + len(sentence) > self.config.chunk_size and current_chunk:
                # Save the current chunk
                chunk = DocumentChunk(
                    content=current_chunk.strip(),
                    source_url=source_url,
                    page_title=page_title,
                    position=chunk_position,
                    metadata={"sentence_count": len(current_chunk.split('. '))}
                )
                chunks.append(chunk)

                # Start a new chunk with the current sentence
                current_chunk = sentence
                chunk_position += 1
            else:
                # Add the sentence to the current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk = DocumentChunk(
                content=current_chunk.strip(),
                source_url=source_url,
                page_title=page_title,
                position=chunk_position,
                metadata={"sentence_count": len(current_chunk.split('. '))}
            )
            chunks.append(chunk)

        self.logger.info(f"Text chunked into {len(chunks)} chunks from {source_url}")
        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences using regex pattern.

        Args:
            text: Text to split into sentences

        Returns:
            List of sentences
        """
        # This regex looks for sentence endings (., !, ?) followed by whitespace and capital letter,
        # or end of string
        sentence_pattern = r'[.!?]+\s+(?=[A-Z])|[.!?]+$'
        sentences = re.split(sentence_pattern, text)

        # Clean up the sentences
        cleaned_sentences = []
        for sentence in sentences:
            # Remove extra whitespace
            sentence = sentence.strip()
            if sentence:
                cleaned_sentences.append(sentence)

        return cleaned_sentences

    def chunk_text_by_paragraph(self, text: str, source_url: str = "", page_title: str = "") -> List[DocumentChunk]:
        """
        Alternative chunking method that splits by paragraphs first.

        Args:
            text: The text to chunk
            source_url: URL where the text originated (for metadata)
            page_title: Title of the source page (for metadata)

        Returns:
            List of DocumentChunk objects
        """
        if not text or len(text.strip()) == 0:
            return []

        # Split by paragraphs first
        paragraphs = text.split('\n\n')
        chunks = []

        for para in paragraphs:
            para = para.strip()
            if not para:
                continue

            # If paragraph is too long, split it further
            if len(para) > self.config.chunk_size:
                sub_chunks = self._split_long_paragraph(para)
                for i, sub_chunk in enumerate(sub_chunks):
                    chunk = DocumentChunk(
                        content=sub_chunk,
                        source_url=source_url,
                        page_title=page_title,
                        position=len(chunks),
                        metadata={"paragraph_part": i, "original_paragraph": True}
                    )
                    chunks.append(chunk)
            else:
                chunk = DocumentChunk(
                    content=para,
                    source_url=source_url,
                    page_title=page_title,
                    position=len(chunks),
                    metadata={"paragraph": True}
                )
                chunks.append(chunk)

        self.logger.info(f"Text chunked by paragraphs into {len(chunks)} chunks from {source_url}")
        return chunks

    def _split_long_paragraph(self, paragraph: str) -> List[str]:
        """
        Split a long paragraph into smaller chunks.

        Args:
            paragraph: Long paragraph to split

        Returns:
            List of smaller text chunks
        """
        sentences = self._split_into_sentences(paragraph)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk) + len(sentence) <= self.config.chunk_size:
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence
            else:
                # Current chunk would be too large, so save it and start a new one
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence

        # Add the final chunk
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def chunk_with_overlap(self, text: str, source_url: str = "", page_title: str = "", overlap: int = 50) -> List[DocumentChunk]:
        """
        Chunk text with overlap to maintain context between chunks.

        Args:
            text: The text to chunk
            source_url: URL where the text originated (for metadata)
            page_title: Title of the source page (for metadata)
            overlap: Number of characters to overlap between chunks

        Returns:
            List of DocumentChunk objects
        """
        if not text or len(text.strip()) == 0:
            return []

        chunks = []
        start_idx = 0
        position = 0

        while start_idx < len(text):
            end_idx = start_idx + self.config.chunk_size

            # If we're at the end, just take the remainder
            if end_idx >= len(text):
                chunk_text = text[start_idx:]
            else:
                # Try to break at sentence boundary if possible
                chunk_text = text[start_idx:end_idx]

            # Create the chunk
            chunk = DocumentChunk(
                content=chunk_text,
                source_url=source_url,
                page_title=page_title,
                position=position,
                metadata={"overlap_chunk": True, "overlap_size": overlap}
            )
            chunks.append(chunk)

            # Move to the next chunk position with overlap
            start_idx = end_idx - overlap if end_idx < len(text) else len(text)
            position += 1

        self.logger.info(f"Text chunked with overlap into {len(chunks)} chunks from {source_url}")
        return chunks


def get_chunker_instance(config) -> TextChunker:
    """
    Factory function to create a TextChunker instance.

    Args:
        config: Configuration object

    Returns:
        TextChunker: Configured TextChunker instance
    """
    return TextChunker(config)


if __name__ == "__main__":
    # Example usage
    from .config import Config
    config = Config(
        cohere_api_key="dummy",
        chunk_size=512
    )

    chunker = get_chunker_instance(config)

    sample_text = """
    This is the first sentence. This is the second sentence. Here's the third sentence.
    This paragraph has multiple sentences that will be chunked appropriately.
    The text chunker ensures that we don't break sentences across chunk boundaries.

    This is a new paragraph. It should be kept together when possible.
    But if it's too long, it will be split appropriately.
    """

    chunks = chunker.chunk_text(sample_text, "https://example.com", "Example Page")

    print(f"Created {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks):
        print(f"Chunk {i}: {len(chunk.content)} chars - {chunk.content[:50]}...")