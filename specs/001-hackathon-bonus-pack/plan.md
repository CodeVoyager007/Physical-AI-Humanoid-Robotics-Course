# Implementation Plan: Hackathon Bonus Pack (Revised)

**Branch**: `001-hackathon-bonus-pack` | **Date**: 2025-12-13 | **Spec**: [specs/001-hackathon-bonus-pack/spec.md](specs/001-hackathon-bonus-pack/spec.md)
**Input**: Feature specification from `specs/001-hackathon-bonus-pack/spec.md`

## Summary

Implement Translation and Chatbot Customization features for the "Sentient Machines" platform. This includes a frontend `ChapterToolbar` with a "Translate" button, and a Chat Widget with dropdowns for user background to customize chatbot responses. The backend will have endpoints for `/translate` and an updated `/chat` to handle this logic.

## Technical Context

**Language/Version**: Python 3.12+, TypeScript
**Primary Dependencies**: FastAPI, Docusaurus v3, openai-agents-sdk, openai-chatkit, React
**Storage**: N/A (No user data to store)
**Testing**: pytest, Jest
**Target Platform**: Web
**Project Type**: Web application
**Performance Goals**: Return translated content and customized chat responses in under 5 seconds.
**Constraints**: Adhere to "Neo-Tech" design system.
**Scale/Scope**: [NEEDS CLARIFICATION: What is the expected number of users and concurrent requests?]

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [ ] **Tech Stack**: Does this plan adhere to the approved frameworks (Docusaurus, FastAPI) and libraries (`openai-agents-sdk`, etc.)?
- [ ] **Feature Requirements**: Does the design cover all non-negotiable features (Translation, Chatbot Customization)?
- [ ] **Architecture**: Does the plan encapsulate backend logic in `Skill` classes?
- [ ] **Design System**: Do all UI components conform to the "Neo-Tech" theme?

## Project Structure

### Documentation (this feature)

```text
specs/001-hackathon-bonus-pack/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── openapi.yml
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
# Web application
backend/
├── src/
│   ├── skills/
│   │   ├── personalize.py
│   │   └── translate.py
│   └── main.py
└── tests/

frontend/
├── src/
│   ├── components/
│   │   └── ChapterToolbar/
│   │       └── index.tsx
│   └── theme/
│       └── ChatWidget.tsx
└── tests/
```

**Structure Decision**: The project is a web application with a clear frontend/backend separation. The frontend is in the `book/` directory (I will use `frontend/` for consistency in the plan) and the backend is in the `rag-chatbot/` directory (I will use `backend/`).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
|           |            |                                     |
