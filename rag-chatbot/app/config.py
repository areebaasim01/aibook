"""
Configuration settings for the RAG Chatbot
Uses pydantic-settings for environment variable management
"""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""
    
    # OpenAI Configuration
    openai_api_key: str | None = None
    embedding_model: str = "text-embedding-3-small"
    chat_model: str = "gpt-4-turbo-preview"
    max_tokens: int = 1500
    temperature: float = 0.1
    
    # Qdrant Configuration
    qdrant_url: str | None = None
    qdrant_api_key: str | None = None
    qdrant_collection: str = "physical-ai-book"
    
    # Neon PostgreSQL Configuration
    neon_database_url: str | None = None
    
    # Application Settings
    environment: str = "development"
    debug: bool = True
    cors_origins: str = "http://localhost:3000,http://localhost:8000"
    
    # RAG Configuration
    top_k_results: int = 5
    chunk_size: int = 500
    chunk_overlap: int = 50
    similarity_threshold: float = 0.7
    
    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins string to list"""
        return [origin.strip() for origin in self.cors_origins.split(",")]
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance"""
    return Settings()
