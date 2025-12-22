---
description: RAG Chatbot Setup - Physical AI Textbook
---

# 🤖 RAG Chatbot Implementation Guide

This workflow guides you through setting up the RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics textbook.

// turbo-all

---/sp.

## Prerequisites

1. **OpenAI API Key**: Get from https://platform.openai.com/api-keys
2. **Qdrant Cloud Account**: Sign up at https://cloud.qdrant.io/
3. **Neon PostgreSQL**: Create account at https://neon.tech/

---

## Step 1: Set Up Cloud Services

### 1.1 Qdrant Cloud
1. Create a cluster in Qdrant Cloud
2. Copy the cluster URL and API key
3. Save in .env file

### 1.2 Neon PostgreSQL
1. Create a new project
2. Copy the connection string
3. Save in .env file

---

## Step 2: Configure Environment

```bash
cd rag-chatbot
cp .env.example .env
```

Edit `.env` with your credentials:
```env
OPENAI_API_KEY=sk-your-key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/db
```

---

## Step 3: Install Dependencies

```bash
cd rag-chatbot
python -m venv venv
.\venv\Scripts\activate  # Windows
pip install -r requirements.txt
```

---

## Step 4: Ingest Documents

```bash
python scripts/ingest_docs.py
```

This will:
- Read all markdown files from `docs/`
- Generate embeddings using OpenAI
- Store in Qdrant vector database

---

## Step 5: Start the API

```bash
uvicorn app.main:app --reload --port 8000
```

API will be available at:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

---

## Step 6: Test the Chatbot

### Full Book Mode
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "mode": "full_book"}'
```

### Selected Text Mode
```bash
curl -X POST http://localhost:8000/chat/selected \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain this in detail",
    "selected_text": "ROS 2 is to robotics what the nervous system is to the human body",
    "selected_source": "/docs/robotic-nervous-system"
  }'
```

---

## Step 7: Verify Frontend Integration

The ChatWidget component is already integrated via `src/theme/Root.tsx`.

Start the Docusaurus dev server:
```bash
npm run start
```

You should see a chat bubble in the bottom-right corner.

---

## Production Deployment

### Option A: Docker
```bash
cd rag-chatbot
docker-compose up -d
```

### Option B: Cloud Run / Vercel
Update the API URL in `src/theme/Root.tsx` to your deployed endpoint.

---

## Architecture Summary

| Component | Technology | Purpose |
|-----------|------------|---------|
| Embeddings | OpenAI text-embedding-3-small | Semantic search |
| Vector DB | Qdrant Cloud | Document retrieval |
| LLM | GPT-4 Turbo | Response generation |
| Database | Neon PostgreSQL | Sessions & history |
| API | FastAPI | Backend server |
| Frontend | React Component | Chat UI |

---

## Troubleshooting

### "No relevant chunks found"
- Ensure documents were ingested: `GET /ingest/status`
- Re-run ingestion if needed

### CORS errors
- Add your domain to `CORS_ORIGINS` in `.env`

### Low confidence scores
- Adjust `SIMILARITY_THRESHOLD` in `.env`
- Consider re-chunking with smaller `CHUNK_SIZE`

---

*Last Updated: December 2024*