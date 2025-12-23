"""
Configuration management for RAG Chatbot.

Loads environment variables and provides type-safe configuration access.
"""
from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Anthropic API
    anthropic_api_key: str = ""

    # OpenRouter API
    openrouter_api_key: str = ""

    # Qdrant Cloud
    qdrant_url: str = ""
    qdrant_api_key: str = ""

    # Neon Serverless Postgres
    database_url: str = ""

    # Application Settings
    log_level: str = "INFO"
    rate_limit_queries_per_hour: int = 100
    retrieval_top_k: int = 5
    retrieval_threshold: float = 0.7
    session_timeout_minutes: int = 30

    # Server Configuration
    host: str = "0.0.0.0"
    port: int = 8000

    # CORS Settings
    cors_origins: str = "http://localhost:3000,http://localhost:8000,https://saimaamjad786.github.io"

    # Observability
    enable_metrics: bool = True
    enable_tracing: bool = False
    jaeger_endpoint: str = "http://localhost:14268/api/traces"

    # Test mode (skip database connection)
    test_mode: bool = False

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


# Global settings instance
settings = Settings()
