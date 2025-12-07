# Tasks: Document RAG Chatbot

**Input**: Design documents from `/specs/001-document-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure.

-   [x] T001 [P] Verify Python 3.12+ and `uv` are installed.
-   [x] T002 [P] Create and populate `.env` file from `.env.example` (or `quickstart.md` guide).
-   [x] T003 [P] Ensure Qdrant and PostgreSQL services are running and accessible.
-   [x] T004 [P] Install all dependencies using `uv pip install -r requirements.txt`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

-   [x] T005 [P] Implement database connection logic in `database.py` to connect to Qdrant.
-   [x] T006 [P] Define core data models for embeddings in `database.py`.
-   [x] T007 Implement settings management in `settings.py` to load environment variables.
-   [x] T008 Implement the basic FastAPI app structure in `main.py`, including CORS and middleware.

---

## Phase 3: User Story 1 - RAG Conversation (Priority: P1) 🎯 MVP

**Goal**: A user can have a conversation with the chatbot, and the chatbot will use the information from the book to answer questions.
**Independent Test**: Can be tested by having a conversation with the chatbot and verifying the answers are accurate based on the book's content.

-   [x] T009 [US1] Implement core RAG retrieval chain logic in `rag_service.py`.
-   [x] T010 [US1] Define the `SystemOperator` agent in `rag_service.py` to handle the chat flow.
-   [x] T011 [US1] Create the `/chat` endpoint in `main.py` that utilizes the `rag_service.py`.

---

## Phase 4: User Story 2 - Contextual Chat (Priority: P2)

**Goal**: A user can select text and have a conversation with the chatbot about the selected text.
**Independent Test**: Can be tested by selecting text, asking a question, and verifying the chatbot's response is based on the selected text.

-   [x] T012 [US2] Extend `rag_service.py` to handle a user-provided context string.
-   [x] T013 [US2] Implement logic to prioritize user-provided context over retrieved document context.
-   [x] T014 [US2] Create the `/context-chat` endpoint in `main.py`.

---

## Phase 5: User Story 3 - Data Ingestion (Priority: P3)

**Goal**: The system can ingest markdown files from the `../book/docs` directory and store them in the vector database.
**Independent Test**: Can be tested by running the ingestion script and verifying the data is present in the vector database.

-   [x] T015 [US3] Implement file scanning logic in `ingest_local.py` to read Markdown files from `../frontend/docs`.
-   [x] T016 [US3] Implement text splitting and embedding generation logic in `ingest_local.py`.
-   [x] T017 [US3] Implement logic to store the generated embeddings into the Qdrant collection via `database.py`.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

-   [x] T018 [P] Add comprehensive error handling to all API endpoints in `main.py`.
-   [x] T019 [P] Add logging to `rag_service.py` and `ingest_local.py` for easier debugging.
-   [x] T020 [P] Write comprehensive `README.md` based on `quickstart.md`.
-   [x] T021 Validate `quickstart.md` instructions by following them in a clean environment.

---

## Dependencies & Execution Order

-   **Phase 1 (Setup)** must be completed first.
-   **Phase 2 (Foundational)** depends on Phase 1 and blocks all user stories.
-   **Phase 3, 4, and 5 (User Stories)** depend on Phase 2. They can be worked on sequentially or in parallel if staffed.
-   **Phase 6 (Polish)** can be addressed after all user stories are complete.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1 (RAG Conversation).
4.  **STOP and VALIDATE**: The core chat functionality should be working.

### Incremental Delivery

1.  Complete MVP (US1).
2.  Add User Story 2 (Contextual Chat).
3.  Add User Story 3 (Data Ingestion).
4.  Complete Phase 6 (Polish).