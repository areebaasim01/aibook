"""
Physical AI & Humanoid Robotics - RAG Chatbot API
Main FastAPI application entry point
"""

from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import get_settings
from .models import HealthResponse
from .routers import chat_router, ingestion_router
from .services import get_database_service, get_vector_store_service


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan events"""
    # Startup
    print("🚀 Starting RAG Chatbot API...")
    
    # Initialize database tables
    db = get_database_service()
    try:
        await db.initialize_tables()
        print("✅ Database initialized")
    except Exception as e:
        print(f"⚠️ Database initialization failed: {e}")
    
    # Ensure vector store collection
    vs = get_vector_store_service()
    try:
        await vs.ensure_collection()
        print("✅ Vector store ready")
    except Exception as e:
        print(f"⚠️ Vector store initialization failed: {e}")
    
    yield
    
    # Shutdown
    print("👋 Shutting down RAG Chatbot API...")
    await db.disconnect()


# Create FastAPI app
settings = get_settings()

app = FastAPI(
    title="Physical AI & Humanoid Robotics - RAG Chatbot",
    description="""
    A Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics textbook.
    
    ## Features
    
    - **Context-Aware Answers**: All responses are grounded in the book's content
    - **Source Citations**: Every answer includes references to the source material
    - **Selected Text Mode**: Ask questions about specific highlighted text
    - **Conversation Memory**: Maintains context across multiple questions
    - **Hallucination-Free**: Strict grounding ensures accurate, reliable answers
    
    ## Technology Stack
    
    - **OpenAI**: Embeddings & Chat completions
    - **Qdrant Cloud**: Vector database for semantic search
    - **Neon PostgreSQL**: Session & conversation storage
    - **FastAPI**: High-performance async API
    """,
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router)
app.include_router(ingestion_router)


@app.get("/", tags=["Root"])
async def root():
    """API root - welcome message"""
    return {
        "message": "Physical AI & Humanoid Robotics - RAG Chatbot API",
        "docs": "/docs",
        "health": "/health"
    }


@app.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check():
    """
    Health check endpoint.
    
    Returns the status of the API and connected services.
    """
    db = get_database_service()
    vs = get_vector_store_service()
    
    db_healthy = await db.health_check()
    vs_healthy = await vs.health_check()
    
    services = {
        "database": "healthy" if db_healthy else "unhealthy",
        "vector_store": "healthy" if vs_healthy else "unhealthy",
        "openai": "configured" if settings.openai_api_key else "missing"
    }
    
    overall_status = "healthy" if all([db_healthy, vs_healthy]) else "degraded"
    
    return HealthResponse(
        status=overall_status,
        version="1.0.0",
        services=services
    )
