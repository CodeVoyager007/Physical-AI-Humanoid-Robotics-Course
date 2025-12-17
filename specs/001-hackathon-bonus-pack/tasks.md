# Tasks: Hackathon Bonus Pack (Revised)

**Input**: Design documents from `specs/001-hackathon-bonus-pack/`
**Prerequisites**: plan.md, spec.md

## Format: `[ID] [P?] [Story] Description`

## Phase 1: Setup (Shared Infrastructure)

- [ ] T001 [P] In `book`, install any necessary packages for UI effects (if any).
- [ ] T002 In `rag-chatbot`, ensure all necessary dependencies for Gemini API and FastAPI are in `pyproject.toml`.

---

## Phase 2: User Story 1 - Translation (Priority: P1)

**Goal**: Implement in-place translation of chapter content to Urdu.
**Independent Test**: A user can click the "Translate to Urdu" button, see the content change to plain text Urdu, and click "Show Original" to revert.

### Implementation for User Story 1

- [ ] T003 [US1] In `rag-chatbot/src/skills/translate.py`, ensure the `TranslationSkill`'s prompt explicitly requests plain text output.
- [ ] T004 [US1] In `rag-chatbot/main.py`, ensure the `/translate` endpoint is present and uses the `TranslationSkill`.
- [ ] T005 [US1] In `book/src/components/ChapterToolbar/index.tsx`, implement the `handleTranslate` function to save original content and replace it with the translated plain text, with a button to toggle back.

---

## Phase 3: User Story 2 - Customized Chatbot (Priority: P2)

**Goal**: Allow users to get customized chatbot responses based on their selected background.
**Independent Test**: A user can select a software and hardware background in the chat widget, and the chatbot's responses will be tailored accordingly.

### Implementation for User Story 2

- [ ] T006 [US2] In `rag-chatbot/main.py`, update the `ChatRequest` model to accept `software_background` and `hardware_background`.
- [ ] T007 [US2] In `rag-chatbot/rag_service.py`, update the `generate_answer` method to incorporate the background information into the system prompt.
- [ ] T008 [US2] In `book/src/theme/ChatWidget.tsx`, add state and UI for "Software Background" and "Hardware Background" dropdowns.
- [ ] T009 [US2] In `book/src/theme/ChatWidget.tsx`, update the `handleSubmit` function to send the selected backgrounds with the chat request.
- [ ] T010 [US2] In `book/src/theme/ChatWidget.module.css`, add styles for the new dropdown selectors.

---

## Phase 4: User Story 3 - Content Personalization (Priority: P3)

**Goal**: Demonstrate content personalization using a placeholder profile.
**Independent Test**: A user can click the "Personalize" button and see a personalized version of the content in an overlay.

### Implementation for User Story 3

- [ ] T011 [US3] In `rag-chatbot/src/skills/personalize.py`, ensure the `PersonalizationSkill` is implemented.
- [ ] T012 [US3] In `rag-chatbot/main.py`, ensure the `/personalize` endpoint is present and uses the `PersonalizationSkill` without authentication.
- [ ] T013 [US3] In `book/src/components/ChapterToolbar/index.tsx`, implement the `handlePersonalize` function to call the backend with a placeholder profile and display the result in an overlay.

---

## Phase 5: Polish & Cross-Cutting Concerns

- [ ] T014 [P] Ensure all new UI elements conform to the "Neo-Tech" glassmorphism design system.
- [ ] T015 [P] Add basic error handling for all API calls in the frontend.
- [ ] T016 [P] Add validation for all incoming data in the backend endpoints using Pydantic.
- [ ] T017 Run through the `quickstart.md` guide to validate the end-to-end workflow.

---

## Dependencies & Execution Order

- **Phase 1** should be done first.
- **Phases 2, 3, and 4** can be worked on in parallel.
- **Phase 5** should be done last.

### Parallel Opportunities

- The three user stories (Translation, Chatbot Customization, and Personalization) can be developed independently of each other.
- Within each story, frontend and backend tasks can be developed in parallel.
