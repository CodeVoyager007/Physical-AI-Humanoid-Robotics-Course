<!--
Sync Impact Report:
- Version change: 1.0.0 -> 1.1.0
- Modified principles:
  - [PRINCIPLE_1_NAME] -> Tech Stack
  - [PRINCIPLE_2_NAME] -> Architecture
  - [PRINCIPLE_3_NAME] -> API Contract
- Added sections: None
- Removed sections: Principles 4, 5, 6 and Sections 2, 3
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# Chatbot Constitution: Agentic RAG System

## Core Principles

### 1. Tech Stack
- **Framework**: FastAPI (Python 3.12+).
- **AI SDK**: `openai-agents-sdk` (Python).
- **Vector DB**: `qdrant-client` (Qdrant Cloud).
- **LLM**: `AsyncOpenAI` client configured for **Google Gemini** (`base_url="https://generativelanguage.googleapis.com/v1beta/openai/"`, model=`gemini-1.5-flash`).

### 2. Architecture
- **Agentic Pattern**: Use the "Agent" and "Tool" primitives from the SDK.
- **Skills**: Logic must be encapsulated in reusable "Skills" (Classes) for Ingestion, Retrieval, and Personalization.

### 3. API Contract
- `POST /ingest`: Scans the local `../book/docs` folder and indexes it.
- `POST /chat`: Standard RAG chat.
- `POST /context-chat`: Accepts `{ selection, message }` for the "Select-to-Ask" feature.

## Governance

This Constitution is the single source of truth for project-wide principles. All development, tooling, and architectural decisions must align with it. Amendments require a documented proposal, review by the core team, and a version bump following semantic versioning rules.

**Version**: 1.1.0 | **Ratified**: 2025-12-03 | **Last Amended**: 2025-12-03