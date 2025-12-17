<!--
---
Sync Impact Report
---
- Version change: 1.0.0 → 1.1.0
- Modified principles:
  - Tech Stack Laws (New)
  - LLM Standards (New)
  - Architecture (New)
- Added sections: None
- Removed sections: None
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->

# Project Constitution: Agentic RAG Backend 

## 1. Governance

- **Constitution Version**: 1.1.0
- **Ratification Date**: 2025-12-07
- **Last Amended Date**: 2025-12-07
- **Amendment Process**: Changes to this constitution must be proposed via a pull request and approved by the project owner.
- **Versioning Policy**: This constitution follows semantic versioning.
- **Compliance**: All code must adhere to the principles outlined in this constitution.

## 2. Principles

### Principle 1: Tech Stack Laws

- **Runtime**: Python 3.12+ using `uv` for package management.
- **Framework**: FastAPI (`main.py`) served by Uvicorn.
- **AI Core**: `openai-agents-sdk` and `openai-chatkit` for orchestration.
- **Database**: `qdrant-client` (Vector DB).

### Principle 2: LLM Standards

- **Provider**: Google Gemini 1.5 Flash.
- **Adapter**: Clients must use `AsyncOpenAI` with `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"`.
- **Embeddings**: `text-embedding-004` via Gemini.

### Principle 3: Architecture

- **Pattern**: Modular architecture (`database.py`, `rag_service.py`, `ingest_local.py`).
- **Ingestion**: Local file scanning of the sibling `../book/docs` directory.
- **Context-Awareness**: The API must support `/context-chat` for handling user text selections.