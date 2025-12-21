"""
Ingestion Router
API endpoints for document ingestion
"""

from fastapi import APIRouter, HTTPException, BackgroundTasks
from ..models import IngestRequest, IngestResponse
from ..services import get_ingestion_service
from pathlib import Path

from ..services import get_embedding_service, get_vector_store_service

router = APIRouter(prefix="/ingest", tags=["Ingestion"])


@router.post("", response_model=IngestResponse)
async def ingest_documents(
    request: IngestRequest,
    background_tasks: BackgroundTasks
) -> IngestResponse:
    """
    Ingest markdown documents into the vector store.
    
    This endpoint processes all markdown files in the docs directory,
    extracts content, generates embeddings, and stores them in Qdrant.
    
    - **docs_path**: Path to the docs directory (default: 'docs')
    - **force_reingest**: If true, re-ingests all documents
    """
    try:
        ingestion_service = get_ingestion_service(request.docs_path)
        response = await ingestion_service.ingest_documents(
            docs_path=request.docs_path,
            force_reingest=request.force_reingest
        )
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingestion error: {str(e)}")


@router.get("/status")
async def ingestion_status():
    """
    Get the current status of the vector store.
    
    Returns information about the indexed documents and chunks.
    """
    try:
        from ..services import get_vector_store_service
        vector_store = get_vector_store_service()
        info = await vector_store.get_collection_info()
        return {
            "status": "ready" if "error" not in info else "error",
            "collection": info
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Status error: {str(e)}")



@router.post("/specify", response_model=IngestResponse)
async def ingest_specify():
    """
    Ingest the content specification file `.agent/workflows/sp.specify.md`
    into the vector store so the chatbot can answer questions about the
    content specification.
    """
    try:
        spec_path = Path(".agent/workflows/sp.specify.md")
        if not spec_path.exists():
            raise HTTPException(status_code=404, detail=f"Spec file not found: {spec_path}")

        # Use ingestion service helpers
        ingestion_service = get_ingestion_service(docs_path=str(spec_path.parent))

        # Process the single file into chunks
        chunks = ingestion_service._process_markdown_file(spec_path)
        if not chunks:
            return IngestResponse(success=False, documents_processed=0, chunks_created=0, message="No content extracted from spec file")

        # Generate embeddings and upsert
        embedding_service = get_embedding_service()
        vector_store = get_vector_store_service()

        texts = [c["content"] for c in chunks]
        embeddings = await embedding_service.embed_texts(texts)
        stored = await vector_store.upsert_chunks(chunks, embeddings)

        return IngestResponse(success=True, documents_processed=1, chunks_created=stored, message=f"Ingested spec file: {spec_path}")

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingest specify error: {str(e)}")
