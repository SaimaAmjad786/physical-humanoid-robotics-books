---
title: RAG Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
---

# RAG Chatbot API

A FastAPI-based RAG (Retrieval-Augmented Generation) chatbot for book content.

## Features

- Query book content using natural language
- Vector-based semantic search with Qdrant
- LLM-powered answer generation
- Session management and rate limiting

## Environment Variables

Set these as secrets in your Hugging Face Space:

- `QDRANT_URL`: Your Qdrant Cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `DATABASE_URL`: PostgreSQL connection string (optional)
- `OPENROUTER_API_KEY`: OpenRouter API key for LLM
- `TEST_MODE`: Set to "true" to use mock services

## API Endpoints

- `GET /`: Health check
- `POST /v1/query`: Submit a query
- `GET /health`: Detailed health status
