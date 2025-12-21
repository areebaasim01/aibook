"""
Chat Router
API endpoints for RAG chatbot interactions
"""

from fastapi import APIRouter, HTTPException, Depends
from ..models import ChatRequest, ChatResponse, ChatMode
from ..services import get_rag_pipeline, RAGPipeline

router = APIRouter(prefix="/chat", tags=["Chat"])


@router.post("", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    rag: RAGPipeline = Depends(get_rag_pipeline)
) -> ChatResponse:
    """
    Send a chat message and get a response from the RAG system.
    
    The chatbot will answer based on the Physical AI & Humanoid Robotics textbook content only.
    
    - **message**: Your question about the book content
    - **session_id**: Optional session ID for conversation continuity
    - **mode**: 'full_book' to search entire book, 'selected_text' for specific text
    """
    try:
        response = await rag.chat(
            query=request.message,
            session_id=request.session_id,
            mode=request.mode,
            selected_text=request.selected_text,
            selected_source=request.selected_source
        )
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chat error: {str(e)}")


@router.post("/selected", response_model=ChatResponse)
async def chat_selected_text(
    request: ChatRequest,
    rag: RAGPipeline = Depends(get_rag_pipeline)
) -> ChatResponse:
    """
    Chat about user-selected text from the book.
    
    This mode ensures the response is based ONLY on the selected text,
    providing precise, context-aware answers without hallucination.
    
    - **message**: Your question about the selected text
    - **selected_text**: The text you selected from the book (required)
    - **selected_source**: The page/section where the text was selected
    """
    if not request.selected_text:
        raise HTTPException(
            status_code=400,
            detail="selected_text is required for this endpoint"
        )
    
    try:
        response = await rag.chat(
            query=request.message,
            session_id=request.session_id,
            mode=ChatMode.SELECTED_TEXT,
            selected_text=request.selected_text,
            selected_source=request.selected_source
        )
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chat error: {str(e)}")
