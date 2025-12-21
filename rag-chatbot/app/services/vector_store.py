"""
Qdrant Vector Store Service
Handles vector storage and semantic search using Qdrant Cloud
"""

from typing import List, Optional, Dict, Any
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import uuid

from ..config import get_settings
from ..models import SourceChunk


class VectorStoreService:
    """Service for vector storage and retrieval using Qdrant"""
    
    def __init__(self):
        settings = get_settings()
        self.client = AsyncQdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection
        self.vector_size = 1536  # OpenAI text-embedding-3-small dimension
        self.similarity_threshold = settings.similarity_threshold
        self._configured = bool(settings.qdrant_url and settings.qdrant_api_key)
    
    async def ensure_collection(self) -> bool:
        """Ensure collection exists, create if not"""
        if not self._configured:
            print("⚠️ Qdrant not configured — skipping collection creation")
            return False

        try:
            collections = await self.client.get_collections()
            collection_names = [c.name for c in collections.collections]
            
            if self.collection_name not in collection_names:
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )
                
                # Create payload indexes for filtering
                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="source",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="page_title",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
                
            return True
        except Exception as e:
            print(f"Error ensuring collection: {e}")
            return False
    
    async def upsert_chunks(
        self,
        chunks: List[Dict[str, Any]],
        embeddings: List[List[float]]
    ) -> int:
        """Upsert document chunks with embeddings"""
        if len(chunks) != len(embeddings):
            raise ValueError("Chunks and embeddings must have same length")
        
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = str(uuid.uuid4())
            points.append(
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": chunk["content"],
                        "source": chunk["source"],
                        "page_title": chunk["page_title"],
                        "chunk_index": chunk.get("chunk_index", i),
                        "metadata": chunk.get("metadata", {})
                    }
                )
            )
        
        # Upsert in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            await self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
        
        return len(points)
    
    async def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        source_filter: Optional[str] = None,
        score_threshold: Optional[float] = None
    ) -> List[SourceChunk]:
        """Search for similar chunks"""
        threshold = score_threshold or self.similarity_threshold
        
        # Build filter if source specified
        query_filter = None
        if source_filter:
            query_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="source",
                        match=models.MatchValue(value=source_filter)
                    )
                ]
            )
        
        if not self._configured:
            return []

        results = await self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=query_filter,
            limit=top_k,
            score_threshold=threshold
        )
        
        chunks = []
        for result in results:
            chunk = SourceChunk(
                content=result.payload["content"],
                source=result.payload["source"],
                page_title=result.payload["page_title"],
                relevance_score=result.score,
                chunk_id=str(result.id)
            )
            chunks.append(chunk)
        
        return chunks
    
    async def delete_by_source(self, source: str) -> int:
        """Delete all chunks from a specific source"""
        if not self._configured:
            return 0

        result = await self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source",
                            match=models.MatchValue(value=source)
                        )
                    ]
                )
            )
        )
        return result.status
    
    async def get_collection_info(self) -> Dict[str, Any]:
        """Get collection statistics"""
        try:
            if not self._configured:
                return {"error": "qdrant not configured"}

            info = await self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status.value
            }
        except Exception as e:
            return {"error": str(e)}
    
    async def health_check(self) -> bool:
        """Check if Qdrant is accessible"""
        try:
            if not self._configured:
                return False

            await self.client.get_collections()
            return True
        except Exception:
            return False


# Singleton instance
_vector_store_service = None


def get_vector_store_service() -> VectorStoreService:
    """Get singleton vector store service instance"""
    global _vector_store_service
    if _vector_store_service is None:
        _vector_store_service = VectorStoreService()
    return _vector_store_service
