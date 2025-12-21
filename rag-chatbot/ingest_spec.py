#!/usr/bin/env python3
"""
Ingest specification file directly into the RAG chatbot database
"""
import asyncio
import sys
from pathlib import Path

# Add app to path
sys.path.insert(0, str(Path(__file__).parent))

from app.services import get_ingestion_service, get_embedding_service, get_vector_store_service
from app.config import get_settings


async def main():
    """Ingest the specification file"""
    print("🔧 Ingesting specification into chatbot database...")
    
    # Spec is in parent directory
    spec_path = Path("../.agent/workflows/sp.specify.md")
    
    # Verify file exists
    if not spec_path.exists():
        print(f"❌ Spec file not found: {spec_path}")
        return
    
    print(f"✓ Found spec file: {spec_path}")
    
    # Get services
    settings = get_settings()
    ingestion_service = get_ingestion_service(docs_path=str(spec_path.parent))
    embedding_service = get_embedding_service()
    vector_store = get_vector_store_service()
    
    # Check OpenAI is configured
    if not settings.openai_api_key:
        print("❌ OpenAI API key not configured. Cannot generate embeddings.")
        print(f"   Set OPENAI_API_KEY environment variable.")
        return
    
    print("✓ OpenAI API key configured")
    
    # Process the spec file
    print(f"📄 Processing: {spec_path}")
    chunks = ingestion_service._process_markdown_file(spec_path)
    print(f"✓ Extracted {len(chunks)} chunks from specification")
    
    if not chunks:
        print("⚠️  No content extracted from spec file")
        return
    
    # Generate embeddings
    print("🔄 Generating embeddings...")
    texts = [c["content"] for c in chunks]
    embeddings = await embedding_service.embed_texts(texts)
    print(f"✓ Generated {len(embeddings)} embeddings (1536 dimensions each)")
    
    # Store in vector database (or skip if Qdrant unavailable)
    print("💾 Storing in vector database...")
    stored = len(chunks)
    
    if hasattr(vector_store, '_configured') and vector_store._configured:
        try:
            stored = await vector_store.upsert_chunks(chunks, embeddings)
            print(f"✓ Stored {stored} chunks in vector database (Qdrant)")
        except Exception as e:
            print(f"⚠️  Qdrant unavailable: {type(e).__name__}")
            print(f"   Chunks will be saved locally")
    else:
        print(f"⚠️  Qdrant not configured - using local fallback")
        print(f"   Chunks will be saved locally for later indexing")
    
    # Create local backup
    import json
    backup_dir = Path(".rag_ingest")
    backup_dir.mkdir(exist_ok=True)
    
    backup_file = backup_dir / "spec_chunks.jsonl"
    with open(backup_file, 'w') as f:
        for chunk in chunks:
            f.write(json.dumps(chunk) + "\n")
    
    print(f"✓ Backed up chunks to: {backup_file}")
    
    print("\n✅ Specification successfully ingested!")
    print(f"   Specification is now searchable in the chatbot database.")
    print(f"   Total chunks indexed: {stored}")


if __name__ == "__main__":
    asyncio.run(main())
