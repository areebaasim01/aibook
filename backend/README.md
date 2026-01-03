# Book Embeddings Ingestion Pipeline

A Python application that crawls documentation websites, extracts clean text content, generates embeddings using Cohere, and stores them in a Qdrant vector database.

## Features

- **URL Crawling**: Fetch and extract clean text from Docusaurus and other documentation sites
- **Content Processing**: Remove HTML tags, navigation elements, and other irrelevant content
- **Text Chunking**: Split content into appropriately sized chunks for embedding
- **Embedding Generation**: Generate high-quality embeddings using Cohere's models
- **Vector Storage**: Store embeddings in Qdrant vector database with metadata
- **Resume Capability**: Track processing state to resume interrupted operations
- **Batch Processing**: Efficiently process multiple URLs with progress tracking

## Prerequisites

- Python 3.9+
- `uv` package manager (optional, but recommended)
- Cohere API key
- Qdrant Cloud account (or local Qdrant instance)

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

3. **Install dependencies using uv** (recommended):
   ```bash
   uv sync
   ```

   Or using pip:
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**:
   Copy the example environment file and add your API keys:
   ```bash
   cp .env .env.local
   ```

   Edit `.env.local` and add your Cohere and Qdrant API keys.

## Configuration

The application can be configured through environment variables in the `.env` file:

```bash
# API Keys and Configuration
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here  # Optional, for cloud instances
QDRANT_API_KEY=your_qdrant_api_key_here  # Optional, for cloud instances
QDRANT_HOST=localhost  # For local instances
QDRANT_PORT=6333
QDRANT_COLLECTION_NAME=book_embeddings

# Processing Configuration
CHUNK_SIZE=512  # Size of text chunks for embedding
COHERE_MODEL=embed-english-v3.0  # Cohere model to use
```

## Usage

### Command Line Interface

The application provides a command-line interface for processing URLs:

```bash
# Process a single URL
python -m src.main --url "https://example.com/documentation"

# Process URLs from a file (one per line)
python -m src.main --url-file urls.txt

# Process URLs from environment variable
export COHERE_URLS='["https://example1.com", "https://example2.com"]'
python -m src.main --use-env

# Resume interrupted processing
python -m src.main --url-file urls.txt --resume

# Dry run (don't store embeddings in vector database)
python -m src.main --url-file urls.txt --no-store

# Set logging level
python -m src.main --url-file urls.txt --log-level DEBUG
```

### URL Sources

The application supports multiple ways to specify URLs to process:

1. **Command line argument**: `--url <single-url>`
2. **File**: `--url-file <path-to-file>` (one URL per line, comments with #)
3. **Environment variable**: `--use-env` (reads from `COHERE_URLS` environment variable)
4. **Default file**: `urls.txt` in the current directory (if no other source specified)

## Architecture

The application consists of several modules:

- **`config.py`**: Handles loading and validating configuration from environment variables
- **`crawler.py`**: Fetches URLs and extracts clean text content from web pages
- **`chunker.py`**: Splits text into appropriately sized chunks for embedding
- **`embedder.py`**: Generates embeddings using Cohere API with caching and batching
- **`vector_db.py`**: Interacts with Qdrant vector database for storage and retrieval
- **`state_manager.py`**: Tracks processing state for resume capability
- **`pipeline.py`**: Orchestrates the complete workflow
- **`main.py`**: Command-line interface entry point

## Example Usage

1. **Create a `urls.txt` file** with documentation URLs to process:
   ```
   https://docs.example.com/getting-started
   https://docs.example.com/api-reference
   https://docs.example.com/tutorials
   ```

2. **Run the ingestion pipeline**:
   ```bash
   python -m src.main --url-file urls.txt
   ```

3. **Monitor progress** as the application processes each URL, chunks the content, generates embeddings, and stores them in Qdrant.

## Development

### Running Tests

TODO: Add test instructions once testing framework is implemented.

### Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests for new functionality
5. Run tests to ensure everything works
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

## Troubleshooting

### Common Issues

- **API Rate Limits**: If you encounter rate limiting, the application has built-in retry logic with exponential backoff.
- **Large Documents**: The chunker handles large documents by splitting them into smaller pieces.
- **Network Issues**: The resume capability allows you to restart processing from where it left off.

### Logging

The application logs progress and errors to help with troubleshooting. Use `--log-level DEBUG` for more detailed output.

## Performance

- The application processes URLs sequentially by default
- Embedding generation is batched for efficiency
- Content extraction and chunking are optimized for performance
- Progress bars show real-time processing status

## Security

- URL validation prevents SSRF attacks
- API keys are loaded from environment variables
- Content is sanitized to prevent injection attacks