---
id: 001
title: Constitution Update
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-pro
feature: none
branch: main
user: user
command: /sp.constitution
labels: [documentation,constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:

---

## Prompt

/sp.constitution # Project Constitution: Agentic RAG Backend (As-Built)

**Context:** Documenting the established standards for the existing "rag-chatbot" project.

1. **Tech Stack Laws**:
   - **Runtime**: Python 3.12+ using `uv` for package management.
   - **Framework**: FastAPI (`main.py`) served by Uvicorn.
   - **AI Core**: `openai-agents-sdk` and `openai-chatkit` for orchestration.
   - **Database**: `qdrant-client` (Vector DB) and `sqlalchemy` (Neon Postgres).

2. **LLM Standards**:
   - **Provider**: Google Gemini 1.5 Flash.
   - **Adapter**: Clients must use `AsyncOpenAI` with `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"`.
   - **Embeddings**: `text-embedding-004` via Gemini.

3. **Architecture**:
   - **Pattern**: Modular architecture (`database.py`, `rag_service.py`, `ingest_local.py`).
   - **Ingestion**: Local file scanning of the sibling `../book/docs` directory.
   - **Context-Awareness**: The API must support `/context-chat` for handling user text selections.

## Response snapshot

Constitution updated to v1.1.0, documenting the project's as-built standards for tech stack, LLM usage, and architecture.

Suggested commit message:
`docs: establish constitution v1.1.0 (as-built)`

## Outcome

- ✅ Impact: High
- 🧪 Tests: N/A
- 📁 Files: 1 file updated
- 🔁 Next prompts: None
- 🧠 Reflection: The constitution was updated to reflect the as-built state of the project.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): None
