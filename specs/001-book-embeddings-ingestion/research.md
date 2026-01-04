# Research Findings: URL Ingestion & Embedding Pipeline

## Research Task 1: Optimal Chunk Size
**Decision**: Use 512 tokens as the default chunk size
**Rationale**: Cohere's models handle up to 512 tokens well, providing good semantic coherence while allowing for efficient processing. This size balances context retention with processing efficiency.
**Alternatives considered**:
- 256 tokens: Smaller chunks but potentially lose context
- 1024 tokens: Larger chunks but may exceed some model limits
- Adaptive chunking: Complex to implement, minimal benefit

## Research Task 2: Cohere Model Selection
**Decision**: Use "embed-english-v3.0" model
**Rationale**: This is Cohere's latest and most capable embedding model, optimized for retrieval tasks. It provides excellent performance for semantic search applications.
**Alternatives considered**:
- "embed-multilingual-v3.0": Only needed for multilingual content
- "embed-english-light-v3.0": Less capable but faster/cheaper
- Previous versions: Outdated with better alternatives available

## Research Task 3: Qdrant Configuration
**Decision**: Create collection with 1024-dimensional vectors, cosine distance metric
**Rationale**: Cohere's embed-english-v3.0 produces 1024-dimensional vectors. Cosine distance is optimal for semantic similarity in embedding spaces.
**Configuration**:
- Vector size: 1024
- Distance metric: Cosine
- Collection name: "book_embeddings"
- Additional payload fields: source_url, page_title, section, position, content

## Research Task 4: URL Source Strategy
**Decision**: Support both file-based input and environment variable configuration
**Rationale**: Provides flexibility for different use cases while maintaining simplicity
**Implementation**:
- Default: Read from "urls.txt" file in the backend directory
- Alternative: Read from COHERE_URLS environment variable (JSON array)
- Fallback: Single URL from command line argument

## Additional Research Findings

### Error Handling Strategy
- Implement circuit breaker pattern for external API calls
- Add exponential backoff for retry logic
- Create persistent state tracking to enable resume functionality

### Performance Optimization
- Use batch processing for Cohere API calls (up to 96 texts per request)
- Implement connection pooling for HTTP requests
- Add caching for successful URL fetches to avoid re-processing

### Security Considerations
- Validate all URLs to prevent SSRF attacks
- Sanitize extracted content to prevent injection attacks
- Use secure environment variable handling for API keys