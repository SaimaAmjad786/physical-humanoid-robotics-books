"""Simple RAG Chatbot API for Hugging Face Spaces"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    query: str

class Citation(BaseModel):
    chapter: str
    chapter_num: int = 0
    section: str
    chunk_id: str = ""

class QueryResponse(BaseModel):
    answer: str
    citations: List[Citation]
    response_time_ms: int
    selected_text_mode: bool = False
    retrieval_scores: Optional[List[float]] = None

@app.get("/")
def root():
    return {"service": "RAG Chatbot API", "version": "1.0.0", "status": "running"}

@app.get("/health")
def health():
    return {"status": "healthy"}

@app.post("/v1/query", response_model=QueryResponse)
def query(request: QueryRequest):
    # Simple response for now - can be enhanced with actual RAG later
    return QueryResponse(
        answer=f"I received your question: '{request.query}'. This is a demo response. Please configure the full RAG system for real answers.",
        citations=[Citation(chapter="Demo", section="Introduction", chapter_num=1, chunk_id="demo-1")],
        response_time_ms=100,
        selected_text_mode=False
    )

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 7860))
    uvicorn.run(app, host="0.0.0.0", port=port)
