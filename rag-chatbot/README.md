# RAG Chatbot Backend

## Physical AI & Humanoid Robotics Textbook

A Retrieval-Augmented Generation (RAG) chatbot that answers questions strictly from the book's content.

## Technology Stack

- **FastAPI** - High-performance Python API framework
- **OpenAI** - Embeddings & Chat completions via Agents SDK
- **Qdrant Cloud** - Vector database for semantic search
- **Neon PostgreSQL** - Serverless database for sessions & history
- **Docusaurus** - Frontend integration

## Setup

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
.\venv\Scripts\activate   # Windows

# Install dependencies
pip install -r requirements.txt

# Set environment variables
cp .env.example .env
# Edit .env with your API keys

# Run the server
uvicorn app.main:app --reload
```

## Environment Variables

```env
OPENAI_API_KEY=your-openai-api-key
QDRANT_URL=your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
NEON_DATABASE_URL=postgres://user:pass@ep-xxx.neon.tech/dbname
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/chat` | POST | Send a chat message |
| `/chat/selected` | POST | Chat about selected text |
| `/ingest` | POST | Ingest markdown documents |
| `/health` | GET | Health check |

## Architecture

The RAG pipeline:
1. User query → OpenAI embeddings
2. Vector search in Qdrant
3. Retrieve top-k relevant chunks
4. Augment prompt with context
5. Generate response via OpenAI
6. Return with source citations
