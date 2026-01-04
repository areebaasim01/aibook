# Implementation Plan: URL Ingestion & Embedding Pipeline

**Feature**: 001-book-embeddings-ingestion
**Created**: 2026-01-03
**Status**: Draft
**Input**: Spec-1: URL Ingestion & Embedding Pipeline

## Technical Context

### Architecture Overview
- **Backend**: Python application using Cohere for embeddings and Qdrant for vector storage
- **Frontend**: None (command-line tool only)
- **Infrastructure**: Qdrant Cloud (Free Tier)
- **Deployment**: Standalone Python script for ingestion pipeline

### Technology Stack
- **Language**: Python 3.9+
- **Package Manager**: uv (for fast dependency management)
- **Web Scraping**: requests and BeautifulSoup4 for URL fetching and content extraction
- **Text Processing**: Custom chunking logic with configurable chunk sizes
- **Embeddings**: Cohere Python SDK for generating embeddings
- **Vector Database**: Qdrant Python client for storage and indexing
- **Configuration**: Environment variables and YAML config files

### Dependencies
- **cohere**: For generating text embeddings
- **qdrant-client**: For interacting with Qdrant vector database
- **requests**: For fetching URLs
- **beautifulsoup4**: For HTML parsing and content extraction
- **python-dotenv**: For environment variable management
- **pyyaml**: For configuration file handling
- **tqdm**: For progress indication during processing

### Unknowns/Decisions Needed
- **Chunk Size**: Use 512 tokens as the default chunk size based on Cohere's optimal handling
- **Cohere Model**: Use "embed-english-v3.0" model as the latest and most capable for retrieval tasks
- **Qdrant Collection**: Create collection named "book_embeddings" with 1024-dimensional vectors using cosine distance
- **URL List**: Support both file-based input (urls.txt) and environment variable (COHERE_URLS) sources

## Constitution Check

### Library-First Approach
- The ingestion pipeline will be structured as a reusable library with a main function for execution
- Each component (crawling, chunking, embedding, storage) will be independently testable

### CLI Interface
- The application will expose functionality via a CLI interface
- Will support both human-readable and JSON output formats
- Will accept configuration via command-line arguments and environment variables

### Test-First (NON-NEGOTIABLE)
- Unit tests will be written for each component before implementation
- Integration tests will verify the complete pipeline
- Mock services will be used for external dependencies (Cohere, Qdrant)

### Integration Testing
- Tests will verify the complete ingestion pipeline from URL to stored embeddings
- Mock implementations will be created for external services during testing

### Observability
- Structured logging will be implemented for debugging and monitoring
- Progress indicators will be provided for long-running operations

## Design Gates

### Gate 1: Architecture Feasibility
✅ **PASSED** - Architecture is technically feasible with available technologies

### Gate 2: Resource Constraints
✅ **PASSED** - Solution fits within Qdrant Cloud Free Tier limits

### Gate 3: Performance Requirements
✅ **PASSED** - Design can handle 1000 pages within 2-hour timeframe

### Gate 4: Security Considerations
✅ **PASSED** - No sensitive data processing, using secure API connections

## Phase 0: Research & Resolution

### Research Task 1: Optimal Chunk Size
**Objective**: Determine the optimal chunk size for text processing with Cohere embeddings
**Research**: Investigate Cohere's token limits and best practices for chunking
**Expected Outcome**: Recommended chunk size (likely 512-1024 tokens)

### Research Task 2: Cohere Model Selection
**Objective**: Select the appropriate Cohere embedding model
**Research**: Compare available Cohere models for quality vs. cost considerations
**Expected Outcome**: Specific model name (e.g., "embed-english-v3.0")

### Research Task 3: Qdrant Configuration
**Objective**: Determine optimal Qdrant collection configuration
**Research**: Investigate vector dimensions for Cohere models and optimal indexing
**Expected Outcome**: Collection schema and configuration parameters

### Research Task 4: URL Source Strategy
**Objective**: Define how Docusaurus URLs will be provided to the system
**Research**: Consider various input methods (file, API, configuration)
**Expected Outcome**: Standardized input format for URL lists

## Phase 1: Design & Contracts

### Data Model: Document Chunk
- **id**: Unique identifier for the chunk
- **content**: The text content of the chunk
- **source_url**: URL where the content was found
- **page_title**: Title of the source page
- **section**: Section/heading where the content appears
- **position**: Position of the chunk within the original document
- **embedding**: Vector representation of the content
- **created_at**: Timestamp of when the chunk was processed

### Data Model: Processing State
- **url**: The URL being processed
- **status**: Current status (pending, processing, completed, failed)
- **last_processed**: Timestamp of last processing attempt
- **retry_count**: Number of retry attempts
- **error_message**: Any error encountered during processing

### API Contract: Embedding Generation
```
Function: generate_embeddings(text_chunks: List[str]) -> List[List[float]]
Input: List of text chunks to embed
Output: List of embedding vectors (each a list of floats)
```

### API Contract: Content Extraction
```
Function: extract_content_from_url(url: str) -> str
Input: URL to extract content from
Output: Clean text content from the page
```

### API Contract: Chunking
```
Function: chunk_text(text: str, chunk_size: int) -> List[str]
Input: Text to chunk and desired chunk size
Output: List of text chunks
```

## Phase 2: Implementation Plan

### Task 1: Project Setup
- Create `backend/` directory
- Initialize project with `uv`
- Set up requirements and dependencies
- Configure environment variables handling

### Task 2: URL Fetching Module
- Implement URL fetching with error handling
- Create content extraction using BeautifulSoup
- Add support for different content types
- Implement retry logic for failed requests

### Task 3: Text Processing Module
- Create text cleaning functionality
- Implement chunking logic with configurable size
- Add duplicate detection and removal
- Add content validation and filtering

### Task 4: Embedding Generation Module
- Integrate Cohere API for embeddings
- Implement batch processing for efficiency
- Add error handling for API limits
- Implement caching to avoid redundant calls

### Task 5: Vector Storage Module
- Integrate Qdrant client
- Implement vector storage with metadata
- Create search functionality for validation
- Add error handling for storage operations

### Task 6: Main Pipeline Integration
- Create main function orchestrating the full pipeline
- Add configuration management
- Implement progress tracking and logging
- Add resume capability for interrupted processing

## Success Criteria Alignment

### SC-001: URL Crawling Performance
- Implement concurrent URL fetching to achieve 30-minute target
- Add progress indicators for monitoring

### SC-002: Embedding Success Rate
- Implement robust error handling for 99% success rate
- Add retry mechanisms for transient failures

### SC-003: Storage Success Rate
- Implement validation after storage for 99% success rate
- Add error recovery mechanisms

### SC-004: Search Validation
- Implement test queries to validate relevance
- Add evaluation metrics for accuracy assessment

### SC-005: Pipeline Performance
- Optimize for 1000 pages in 2 hours with parallel processing
- Add performance monitoring and reporting

## Risk Analysis

### Risk 1: API Rate Limits
**Impact**: Could slow down processing significantly
**Mitigation**: Implement rate limiting and retry logic with exponential backoff

### Risk 2: Large Document Processing
**Impact**: Could exceed Cohere token limits
**Mitigation**: Implement chunking that respects token limits

### Risk 3: Qdrant Storage Limits
**Impact**: Could hit Free Tier limits
**Mitigation**: Monitor storage usage and provide warnings

### Risk 4: Network Issues
**Impact**: Could cause incomplete processing
**Mitigation**: Implement resume capability and persistent state tracking