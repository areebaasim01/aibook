"""
Pydantic models for RAG Chatbot API
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from enum import Enum


class ChatMode(str, Enum):
    """Chat mode - full book or selected text only"""
    FULL_BOOK = "full_book"
    SELECTED_TEXT = "selected_text"


class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    message: str = Field(..., min_length=1, max_length=2000, description="User's question")
    session_id: Optional[str] = Field(None, description="Session ID for conversation continuity")
    mode: ChatMode = Field(default=ChatMode.FULL_BOOK, description="Chat mode")
    selected_text: Optional[str] = Field(None, description="Selected text for context (required in selected_text mode)")
    selected_source: Optional[str] = Field(None, description="Source page/section of selected text")
    
    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is ROS 2 and why is it important for robotics?",
                "session_id": "abc123",
                "mode": "full_book"
            }
        }


class SourceChunk(BaseModel):
    """A chunk of source text with metadata"""
    content: str
    source: str
    page_title: str
    relevance_score: float
    chunk_id: str


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    answer: str = Field(..., description="Generated answer from the book content")
    sources: List[SourceChunk] = Field(default=[], description="Source chunks used for the answer")
    session_id: str = Field(..., description="Session ID for conversation continuity")
    confidence: float = Field(..., ge=0, le=1, description="Confidence score of the answer")
    mode: ChatMode = Field(..., description="Mode used for this response")
    
    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 (Robot Operating System 2) is a middleware framework...",
                "sources": [
                    {
                        "content": "ROS 2 is to robotics what the nervous system is to the human body...",
                        "source": "/docs/robotic-nervous-system/ros2-fundamentals",
                        "page_title": "ROS 2 Fundamentals",
                        "relevance_score": 0.95,
                        "chunk_id": "chunk_001"
                    }
                ],
                "session_id": "abc123",
                "confidence": 0.92,
                "mode": "full_book"
            }
        }


class DocumentChunk(BaseModel):
    """A document chunk for ingestion"""
    content: str
    source: str
    page_title: str
    chunk_index: int
    total_chunks: int
    metadata: dict = Field(default_factory=dict)


class IngestRequest(BaseModel):
    """Request model for document ingestion"""
    docs_path: str = Field(default="docs", description="Path to docs directory")
    force_reingest: bool = Field(default=False, description="Force re-ingestion of all documents")


class IngestResponse(BaseModel):
    """Response model for document ingestion"""
    success: bool
    documents_processed: int
    chunks_created: int
    message: str


class ConversationMessage(BaseModel):
    """A message in the conversation history"""
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime
    sources: Optional[List[SourceChunk]] = None


class HealthResponse(BaseModel):
    """Health check response"""
    status: str
    version: str
    services: dict
