"""
OpenAI Embedding Service
Generates embeddings for text chunks using OpenAI's embedding models
"""

import asyncio
from typing import List
from openai import AsyncOpenAI
from tenacity import retry, stop_after_attempt, wait_exponential
import tiktoken

from ..config import get_settings


class EmbeddingService:
    """Service for generating text embeddings using OpenAI"""
    
    def __init__(self):
        settings = get_settings()
        self.client = AsyncOpenAI(api_key=settings.openai_api_key) if settings.openai_api_key else None
        self.model = settings.embedding_model
        self.tokenizer = tiktoken.get_encoding("cl100k_base")
        self.max_tokens = 8191  # Max tokens for embedding model

        self._configured = bool(settings.openai_api_key)
    
    def count_tokens(self, text: str) -> int:
        """Count tokens in text"""
        return len(self.tokenizer.encode(text))
    
    def truncate_text(self, text: str, max_tokens: int = None) -> str:
        """Truncate text to max tokens"""
        max_tokens = max_tokens or self.max_tokens
        tokens = self.tokenizer.encode(text)
        if len(tokens) <= max_tokens:
            return text
        return self.tokenizer.decode(tokens[:max_tokens])
    
    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=1, max=10)
    )
    async def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        # Truncate if necessary
        text = self.truncate_text(text)
        if not self._configured or self.client is None:
            raise RuntimeError("OpenAI API key not configured; cannot compute embeddings")

        response = await self.client.embeddings.create(
            model=self.model,
            input=text
        )
        return response.data[0].embedding
    
    async def embed_texts(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        """Generate embeddings for multiple texts with batching"""
        all_embeddings = []
        
        # Process in batches
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            
            # Truncate each text
            batch = [self.truncate_text(text) for text in batch]
            
            if not self._configured or self.client is None:
                raise RuntimeError("OpenAI API key not configured; cannot compute embeddings")

            response = await self.client.embeddings.create(
                model=self.model,
                input=batch
            )

            embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(embeddings)
            
            # Small delay between batches
            if i + batch_size < len(texts):
                await asyncio.sleep(0.1)
        
        return all_embeddings


# Singleton instance
_embedding_service = None


def get_embedding_service() -> EmbeddingService:
    """Get singleton embedding service instance"""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
