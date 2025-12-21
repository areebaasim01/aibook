"""
Ingestion Script
Run this to ingest all documents into the vector store
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services import get_ingestion_service


async def main():
    """Main ingestion function"""
    # Get docs path (relative to project root)
    docs_path = Path(__file__).parent.parent.parent / "docs"
    
    print(f"📚 Ingesting documents from: {docs_path}")
    
    if not docs_path.exists():
        print(f"❌ Docs path not found: {docs_path}")
        return
    
    ingestion_service = get_ingestion_service(str(docs_path))
    result = await ingestion_service.ingest_documents(str(docs_path))
    
    if result.success:
        print(f"✅ {result.message}")
        print(f"   Documents: {result.documents_processed}")
        print(f"   Chunks: {result.chunks_created}")
    else:
        print(f"❌ {result.message}")


if __name__ == "__main__":
    asyncio.run(main())
