# Quickstart: Document RAG Chatbot

This guide provides a quick overview to get the RAG Chatbot running.

## 1. Prerequisites

-   Python 3.12+
-   `uv` for package management
-   Qdrant instance running and accessible
-   Google Gemini API Key and Project ID configured in `.env`

## 2. Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd rag-chatbot
    ```
2.  **Install dependencies**:
    ```bash
    uv pip install -r requirements.txt
    ```
3.  **Configure environment variables**:
    Create a `.env` file in the project root with the following variables:
    ```
    GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
    QDRANT_HOST="your_qdrant_host"
    QDRANT_PORT="your_qdrant_port"
    ```
    *(Note: Replace placeholders with your actual credentials and host/port information.)*

## 3. Data Ingestion

To populate the vector database with your documentation:

1.  Place your Markdown files in the `frontend/docs` directory (sibling to `rag-chatbot` directory).
2.  Run the ingestion script:
    ```bash
    python ingest_local.py
    ```

## 4. Run the Chatbot API

1.  Start the FastAPI application:
    ```bash
    uvicorn main:app --reload
    ```
2.  Access the API documentation at `http://127.0.0.1:8000/docs` to test the endpoints.

## 5. Interact

-   Use the `/chat` endpoint for general RAG conversations.
-   Use the `/context-chat` endpoint to provide specific text selections for focused conversations.
