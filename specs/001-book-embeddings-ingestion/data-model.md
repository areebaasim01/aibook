# Data Model: Book Embeddings Ingestion Pipeline

## Document Chunk Entity

**Description**: Represents a segment of text extracted from a web page that has been processed into an embedding

### Fields
- **id** (string, required)
  - Unique identifier for the chunk
  - Format: UUID or auto-generated key
  - Primary key for the entity

- **content** (string, required)
  - The text content of the chunk
  - Raw text extracted from the source page
  - Max length: 4096 characters (to respect Cohere limits)

- **source_url** (string, required)
  - URL where the content was found
  - Full URL including protocol
  - Used for reference and validation

- **page_title** (string, optional)
  - Title of the source page
  - Extracted from HTML title tag
  - Helps with context during retrieval

- **section** (string, optional)
  - Section/heading where the content appears
  - Extracted from H1/H2/H3 tags
  - Provides hierarchical context

- **position** (integer, required)
  - Position of the chunk within the original document
  - Zero-based index
  - Enables reassembly of document order

- **embedding** (array of floats, required)
  - Vector representation of the content
  - 1024-dimensional array for Cohere embeddings
  - Used for similarity search

- **created_at** (timestamp, required)
  - Timestamp of when the chunk was processed
  - ISO 8601 format
  - Used for tracking and ordering

- **metadata** (object, optional)
  - Additional metadata about the chunk
  - Key-value pairs for extensibility
  - May include tags, categories, etc.

## Processing State Entity

**Description**: Tracks the processing status of URLs to enable resume functionality

### Fields
- **url** (string, required)
  - The URL being processed
  - Primary identifier for tracking

- **status** (string, required)
  - Current status of processing
  - Values: "pending", "processing", "completed", "failed"

- **last_processed** (timestamp, optional)
  - Timestamp of last processing attempt
  - ISO 8601 format

- **retry_count** (integer, optional)
  - Number of retry attempts
  - Defaults to 0

- **error_message** (string, optional)
  - Any error encountered during processing
  - Human-readable error message for debugging

## Qdrant Collection Schema

**Collection Name**: book_embeddings

### Vector Configuration
- **Size**: 1024 (dimension of Cohere embeddings)
- **Distance**: Cosine (optimal for semantic similarity)

### Payload Fields
- **source_url**: String field for URL reference
- **page_title**: String field for page title
- **section**: String field for document section
- **position**: Integer field for document order
- **content**: Text field for the original content
- **created_at**: Timestamp field for tracking

## Relationships
- Each Document Chunk is stored as a single point in the Qdrant collection
- Processing State entities are stored separately for tracking purposes
- Multiple Document Chunks can originate from the same source_url