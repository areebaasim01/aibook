#!/usr/bin/env python3
"""Test OpenAI API key validity"""
import asyncio
from pathlib import Path
import sys

# Add project to path
sys.path.insert(0, str(Path(__file__).parent))

from app.config import get_settings
from app.services import get_embedding_service


async def test_openai():
    settings = get_settings()
    
    if not settings.openai_api_key:
        print("❌ OPENAI_API_KEY is not set in .env")
        return False
    
    print("✓ OPENAI_API_KEY is set")
    
    try:
        service = get_embedding_service()
        if not service._configured:
            print("❌ OpenAI not configured in embedding service")
            return False
        
        # Test with a simple query
        test_text = "test embedding"
        embedding = await service.embed_text(test_text)
        
        if embedding and len(embedding) > 0:
            print(f"✅ OpenAI API works! Generated embedding with {len(embedding)} dimensions")
            return True
        else:
            print("❌ Embedding returned empty or invalid")
            return False
            
    except Exception as e:
        print(f"❌ OpenAI API error: {e}")
        return False


if __name__ == "__main__":
    result = asyncio.run(test_openai())
    sys.exit(0 if result else 1)
