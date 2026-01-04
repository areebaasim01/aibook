# API Contracts: Book Embeddings Ingestion

## Content Extraction Service

### Extract Content from URL
```
Function: extract_content_from_url(url: str) -> dict
```

**Input:**
```json
{
  "url": "https://example.com/documentation/page"
}
```

**Output:**
```json
{
  "success": true,
  "content": "Clean text content extracted from the page...",
  "title": "Page Title",
  "metadata": {
    "url": "https://example.com/documentation/page",
    "fetched_at": "2026-01-03T10:30:00Z",
    "word_count": 1250
  }
}
```

**Errors:**
- 400: Invalid URL format
- 404: URL not found
- 500: Content extraction failed

## Text Processing Service

### Chunk Text
```
Function: chunk_text(text: str, chunk_size: int = 512) -> dict
```

**Input:**
```json
{
  "text": "Long text content to be chunked...",
  "chunk_size": 512
}
```

**Output:**
```json
{
  "chunks": [
    {
      "id": "chunk_001",
      "content": "First chunk of text content...",
      "position": 0
    },
    {
      "id": "chunk_002",
      "content": "Second chunk of text content...",
      "position": 1
    }
  ],
  "total_chunks": 2
}
```

## Embedding Service

### Generate Embeddings
```
Function: generate_embeddings(texts: List[str]) -> dict
```

**Input:**
```json
{
  "texts": [
    "First text to embed",
    "Second text to embed"
  ],
  "model": "embed-english-v3.0"
}
```

**Output:**
```json
{
  "embeddings": [
    [0.1, 0.3, 0.5, ...], // 1024-dimensional vector
    [0.2, 0.4, 0.6, ...]  // 1024-dimensional vector
  ],
  "model": "embed-english-v3.0",
  "total_embeddings": 2
}
```

## Storage Service

### Store Embeddings
```
Function: store_embeddings(embeddings_data: List[dict]) -> dict
```

**Input:**
```json
{
  "embeddings_data": [
    {
      "id": "chunk_001",
      "vector": [0.1, 0.3, 0.5, ...],
      "payload": {
        "content": "Text content...",
        "source_url": "https://example.com/page",
        "page_title": "Page Title",
        "section": "Section Name",
        "position": 0,
        "created_at": "2026-01-03T10:30:00Z"
      }
    }
  ]
}
```

**Output:**
```json
{
  "stored_count": 1,
  "collection": "book_embeddings",
  "success": true
}
```