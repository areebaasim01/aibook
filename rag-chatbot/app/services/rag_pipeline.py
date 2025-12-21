"""
RAG Pipeline Service
Implements the core Retrieval-Augmented Generation logic
"""

from typing import List, Optional, Tuple
from openai import AsyncOpenAI

from ..config import get_settings
from ..models import ChatMode, SourceChunk, ChatResponse
from .embedding_service import get_embedding_service
from .vector_store import get_vector_store_service
from .database_service import get_database_service


class RAGPipeline:
    """
    RAG Pipeline for hallucination-free responses from book content
    
    Architecture:
    1. User query → Embed with OpenAI
    2. Search Qdrant for similar chunks
    3. Build context-augmented prompt
    4. Generate response with strict grounding
    5. Return answer with source citations
    """
    
    SYSTEM_PROMPT = """You are an expert teaching assistant for the "Physical AI & Humanoid Robotics" textbook. 
Your role is to answer questions ONLY based on the provided context from the book.

CRITICAL RULES:
1. ONLY answer based on the provided context. Never make up information.
2. If the context doesn't contain enough information to answer, say "I don't have enough information in the textbook to answer this question."
3. Always cite which section/chapter your answer comes from.
4. Keep answers clear, educational, and technically accurate.
5. If the question is outside the scope of robotics/Physical AI, politely redirect to the book's topics.

CONTEXT FROM THE TEXTBOOK:
{context}

Remember: You can ONLY use information from the context above. Do not use any external knowledge."""

    SELECTED_TEXT_PROMPT = """You are an expert teaching assistant helping with the "Physical AI & Humanoid Robotics" textbook.
The user has selected specific text from the book and has a question about it.

CRITICAL RULES:
1. Answer ONLY based on the selected text provided.
2. If the selected text doesn't contain enough information, say so clearly.
3. Explain concepts in a clear, educational manner.
4. Do not add information not present in the selected text.

SELECTED TEXT (from {source}):
{selected_text}

Answer the user's question based ONLY on this selected text."""

    def __init__(self):
        settings = get_settings()
        self.openai_client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.chat_model = settings.chat_model
        self.max_tokens = settings.max_tokens
        self.temperature = settings.temperature
        self.top_k = settings.top_k_results
        
        self.embedding_service = get_embedding_service()
        self.vector_store = get_vector_store_service()
        self.database = get_database_service()
    
    async def _retrieve_context(
        self,
        query: str,
        top_k: int = None
    ) -> Tuple[List[SourceChunk], str]:
        """Retrieve relevant chunks from vector store"""
        top_k = top_k or self.top_k
        
        # Generate query embedding
        try:
            query_embedding = await self.embedding_service.embed_text(query)
        except Exception:
            # Embedding not available (OpenAI not configured) — return empty context
            return [], ""

        # Search vector store
        try:
            chunks = await self.vector_store.search(
                query_embedding=query_embedding,
                top_k=top_k
            )
        except Exception:
            chunks = []
        
        # Build context string
        context_parts = []
        for i, chunk in enumerate(chunks):
            context_parts.append(
                f"[Source {i+1}: {chunk.page_title}]\n{chunk.content}\n"
            )
        
        context = "\n---\n".join(context_parts)
        
        return chunks, context
    
    async def _generate_response(
        self,
        query: str,
        system_prompt: str,
        conversation_history: List[dict] = None
    ) -> str:
        """Generate response using OpenAI"""
        messages = [{"role": "system", "content": system_prompt}]
        
        # Add conversation history for context
        if conversation_history:
            for msg in conversation_history[-4:]:  # Last 4 messages for context
                messages.append({
                    "role": msg["role"],
                    "content": msg["content"]
                })
        
        # Add current query
        messages.append({"role": "user", "content": query})
        
        response = await self.openai_client.chat.completions.create(
            model=self.chat_model,
            messages=messages,
            max_tokens=self.max_tokens,
            temperature=self.temperature
        )
        
        return response.choices[0].message.content
    
    async def _calculate_confidence(
        self,
        chunks: List[SourceChunk],
        answer: str
    ) -> float:
        """Calculate confidence score based on retrieval quality"""
        if not chunks:
            return 0.0
        
        # Average relevance score of retrieved chunks
        avg_relevance = sum(c.relevance_score for c in chunks) / len(chunks)
        
        # Penalize if answer indicates uncertainty
        uncertainty_phrases = [
            "don't have enough information",
            "not covered in the textbook",
            "cannot find",
            "outside the scope"
        ]
        
        for phrase in uncertainty_phrases:
            if phrase.lower() in answer.lower():
                return min(avg_relevance, 0.3)
        
        return avg_relevance
    
    async def chat(
        self,
        query: str,
        session_id: Optional[str] = None,
        mode: ChatMode = ChatMode.FULL_BOOK,
        selected_text: Optional[str] = None,
        selected_source: Optional[str] = None
    ) -> ChatResponse:
        """
        Main chat function - handles both full book and selected text modes
        """
        # Get or create session
        session_id = await self.database.get_or_create_session(session_id)
        
        # Get conversation history
        history = await self.database.get_conversation_history(session_id)
        
        # Handle based on mode
        if mode == ChatMode.SELECTED_TEXT:
            return await self._chat_selected_text(
                query=query,
                session_id=session_id,
                selected_text=selected_text,
                selected_source=selected_source,
                history=history
            )
        else:
            return await self._chat_full_book(
                query=query,
                session_id=session_id,
                history=history
            )
    
    async def _chat_full_book(
        self,
        query: str,
        session_id: str,
        history: List[dict]
    ) -> ChatResponse:
        """Handle chat using full book context"""
        # Retrieve relevant chunks
        chunks, context = await self._retrieve_context(query)
        
        if not chunks:
            answer = "I couldn't find relevant information in the textbook to answer your question. Could you try rephrasing or asking about a topic covered in the Physical AI & Humanoid Robotics curriculum?"
            confidence = 0.0
        else:
            # Build prompt and generate
            system_prompt = self.SYSTEM_PROMPT.format(context=context)
            answer = await self._generate_response(query, system_prompt, history)
            confidence = await self._calculate_confidence(chunks, answer)
        
        # Save to history
        await self.database.add_message(session_id, "user", query, mode="full_book")
        await self.database.add_message(
            session_id, "assistant", answer,
            sources=[c.model_dump() for c in chunks],
            mode="full_book"
        )
        
        return ChatResponse(
            answer=answer,
            sources=chunks,
            session_id=session_id,
            confidence=confidence,
            mode=ChatMode.FULL_BOOK
        )
    
    async def _chat_selected_text(
        self,
        query: str,
        session_id: str,
        selected_text: str,
        selected_source: str,
        history: List[dict]
    ) -> ChatResponse:
        """Handle chat using only selected text"""
        if not selected_text:
            return ChatResponse(
                answer="Please select some text from the book to ask about.",
                sources=[],
                session_id=session_id,
                confidence=0.0,
                mode=ChatMode.SELECTED_TEXT
            )
        
        # Build prompt with selected text only
        system_prompt = self.SELECTED_TEXT_PROMPT.format(
            selected_text=selected_text,
            source=selected_source or "the textbook"
        )
        
        # Generate response
        answer = await self._generate_response(query, system_prompt, history)
        
        # Create source chunk from selected text
        source_chunk = SourceChunk(
            content=selected_text[:500] + ("..." if len(selected_text) > 500 else ""),
            source=selected_source or "selected_text",
            page_title="Selected Text",
            relevance_score=1.0,
            chunk_id="selected"
        )
        
        # Save to history
        await self.database.add_message(
            session_id, "user", query,
            mode="selected_text"
        )
        await self.database.add_message(
            session_id, "assistant", answer,
            sources=[source_chunk.model_dump()],
            mode="selected_text"
        )
        
        return ChatResponse(
            answer=answer,
            sources=[source_chunk],
            session_id=session_id,
            confidence=0.95,  # High confidence for selected text mode
            mode=ChatMode.SELECTED_TEXT
        )


# Singleton instance
_rag_pipeline = None


def get_rag_pipeline() -> RAGPipeline:
    """Get singleton RAG pipeline instance"""
    global _rag_pipeline
    if _rag_pipeline is None:
        _rag_pipeline = RAGPipeline()
    return _rag_pipeline
