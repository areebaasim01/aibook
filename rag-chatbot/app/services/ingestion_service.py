"""
Document Ingestion Service
Processes markdown files and ingests them into the vector store
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Any, Tuple
import asyncio
from bs4 import BeautifulSoup
import markdown

from ..config import get_settings
from ..models import IngestResponse
from .embedding_service import get_embedding_service
from .vector_store import get_vector_store_service


class IngestionService:
    """Service for ingesting markdown documents into vector store"""
    
    def __init__(self, docs_path: str = None):
        settings = get_settings()
        self.docs_path = docs_path or "docs"
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap
        
        self.embedding_service = get_embedding_service()
        self.vector_store = get_vector_store_service()
        
        self.md = markdown.Markdown(extensions=['fenced_code', 'tables'])
    
    def _extract_frontmatter(self, content: str) -> Tuple[Dict[str, str], str]:
        """Extract YAML frontmatter from markdown content"""
        frontmatter = {}
        body = content
        
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                # Parse simple frontmatter
                for line in parts[1].strip().split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        frontmatter[key.strip()] = value.strip().strip('"\'')
                body = parts[2].strip()
        
        return frontmatter, body
    
    def _clean_markdown(self, content: str) -> str:
        """Convert markdown to plain text"""
        # Convert to HTML first
        html = self.md.convert(content)
        self.md.reset()
        
        # Parse HTML and extract text
        soup = BeautifulSoup(html, 'html.parser')
        
        # Remove code blocks for cleaner text
        for code in soup.find_all(['pre', 'code']):
            code.decompose()
        
        # Get text
        text = soup.get_text(separator=' ')
        
        # Clean up whitespace
        text = re.sub(r'\s+', ' ', text).strip()
        
        return text
    
    def _chunk_text(self, text: str, source: str = None) -> List[Dict[str, Any]]:
        """Split text into overlapping chunks"""
        chunks = []
        words = text.split()
        
        if len(words) <= self.chunk_size:
            chunks.append({
                "content": text,
                "source": source,
                "chunk_index": 0,
                "total_chunks": 1
            })
            return chunks
        
        # Calculate step size (chunk_size - overlap)
        step = max(1, self.chunk_size - self.chunk_overlap)
        total_chunks = (len(words) - self.chunk_overlap) // step + 1
        
        for i in range(0, len(words), step):
            chunk_words = words[i:i + self.chunk_size]
            if len(chunk_words) < self.chunk_size // 2:
                # Skip very small final chunks
                break
                
            chunk_text = ' '.join(chunk_words)
            chunks.append({
                "content": chunk_text,
                "source": source,
                "chunk_index": len(chunks),
                "total_chunks": total_chunks
            })
        
        return chunks
    
    def _process_markdown_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """Process a single markdown file into chunks"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            return []
        
        # Extract frontmatter and body
        frontmatter, body = self._extract_frontmatter(content)
        
        # Get page title
        page_title = frontmatter.get('title', '')
        if not page_title:
            # Try to extract from first heading
            heading_match = re.search(r'^#\s+(.+)$', body, re.MULTILINE)
            if heading_match:
                page_title = heading_match.group(1)
            else:
                page_title = file_path.stem.replace('-', ' ').title()
        
        # Clean markdown
        clean_text = self._clean_markdown(body)
        
        if not clean_text.strip():
            return []
        
        # Generate source path (relative URL)
        relative_path = file_path.relative_to(self.docs_path)
        source = f"/docs/{relative_path.as_posix()}".replace('.md', '')
        
        # Chunk the text
        chunks = self._chunk_text(clean_text, source)
        
        # Add page title to all chunks
        for chunk in chunks:
            chunk["page_title"] = page_title
            chunk["metadata"] = {
                "file_path": str(file_path),
                "frontmatter": frontmatter
            }
        
        return chunks
    
    async def ingest_documents(
        self,
        docs_path: str = None,
        force_reingest: bool = False
    ) -> IngestResponse:
        """Ingest all markdown documents from docs directory"""
        docs_path = Path(docs_path or self.docs_path)
        
        if not docs_path.exists():
            return IngestResponse(
                success=False,
                documents_processed=0,
                chunks_created=0,
                message=f"Docs path not found: {docs_path}"
            )
        
        # Ensure vector store collection exists
        await self.vector_store.ensure_collection()
        
        # Find all markdown files
        md_files = list(docs_path.rglob('*.md'))
        
        if not md_files:
            return IngestResponse(
                success=False,
                documents_processed=0,
                chunks_created=0,
                message="No markdown files found"
            )
        
        all_chunks = []
        docs_processed = 0
        
        for file_path in md_files:
            chunks = self._process_markdown_file(file_path)
            if chunks:
                all_chunks.extend(chunks)
                docs_processed += 1
        
        if not all_chunks:
            return IngestResponse(
                success=False,
                documents_processed=docs_processed,
                chunks_created=0,
                message="No content extracted from documents"
            )
        # If vector store or embedding service not configured, write chunks to local file
        if not getattr(self.vector_store, "_configured", False) or not getattr(self.embedding_service, "_configured", False):
            out_dir = Path(".rag_ingest")
            out_dir.mkdir(parents=True, exist_ok=True)
            out_file = out_dir / "extracted_chunks.jsonl"

            import json
            written = 0
            with open(out_file, "w", encoding="utf-8") as fh:
                for c in all_chunks:
                    fh.write(json.dumps(c, ensure_ascii=False) + "\n")
                    written += 1

            return IngestResponse(
                success=True,
                documents_processed=docs_processed,
                chunks_created=written,
                message=f"Extracted {written} chunks and saved to {out_file}. Configure OpenAI/Qdrant and re-run ingestion to index."
            )

        # Generate embeddings
        print(f"Generating embeddings for {len(all_chunks)} chunks...")
        chunk_texts = [c["content"] for c in all_chunks]
        embeddings = await self.embedding_service.embed_texts(chunk_texts)
        
        # Store in vector database
        print("Storing in Qdrant...")
        chunks_stored = await self.vector_store.upsert_chunks(all_chunks, embeddings)
        
        return IngestResponse(
            success=True,
            documents_processed=docs_processed,
            chunks_created=chunks_stored,
            message=f"Successfully ingested {docs_processed} documents into {chunks_stored} chunks"
        )


# Factory function
def get_ingestion_service(docs_path: str = None) -> IngestionService:
    """Get ingestion service instance"""
    return IngestionService(docs_path)
