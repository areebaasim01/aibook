# Quickstart Guide: Book Embeddings Ingestion Pipeline

## Prerequisites

- Python 3.9+
- `uv` package manager installed
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Create the backend directory:**
   ```bash
   mkdir backend
   cd backend
   ```

2. **Initialize the project with uv:**
   ```bash
   uv init
   ```

3. **Set up environment variables:**
   Create a `.env` file with:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

4. **Install dependencies:**
   ```bash
   uv pip install cohere qdrant-client requests beautifulsoup4 python-dotenv pyyaml tqdm
   ```

## Configuration

1. **Prepare URL list:**
   Create a `urls.txt` file with one URL per line:
   ```
   https://example-docs.com/page1
   https://example-docs.com/page2
   https://example-docs.com/page3
   ```

2. **Alternative URL configuration:**
   Set the `COHERE_URLS` environment variable as a JSON array:
   ```bash
   export COHERE_URLS='["https://example.com/page1", "https://example.com/page2"]'
   ```

## Running the Pipeline

1. **Create the main.py file** with the ingestion pipeline implementation

2. **Run the pipeline:**
   ```bash
   python main.py
   ```

## Expected Output

The pipeline will:
1. Fetch content from each URL
2. Extract clean text content
3. Chunk the text into 512-token segments
4. Generate embeddings using Cohere
5. Store embeddings in Qdrant with metadata
6. Provide progress updates throughout the process

## Verification

After completion, verify the embeddings were stored correctly by:
1. Checking the Qdrant collection "book_embeddings" has the expected number of points
2. Running a test similarity search to confirm embeddings are retrievable