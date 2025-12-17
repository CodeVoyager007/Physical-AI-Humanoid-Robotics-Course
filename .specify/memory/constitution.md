<!--
Sync Impact Report:
- Version change: 1.0.0 -> 1.1.0
- Modified principles:
  - PRINCIPLE_1_NAME -> Tech Stack Integration Laws
  - PRINCIPLE_2_NAME -> Feature Requirements (Non-Negotiable)
  - PRINCIPLE_3_NAME -> Architecture Standards
  - PRINCIPLE_4_NAME -> Design System ("Neo-Tech")
- Added sections: None
- Removed sections: Sections for principles 5 and 6.
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Set the original adoption date.
-->
# Sentient Machines Constitution

## Core Principles

### 1. Tech Stack Integration Laws
- **Frontend (`./book`)**:
  - Framework: Docusaurus v3 (TypeScript).
  - Auth: **Better-Auth** (Client-side integration).
  - State: React Context for managing User Profile (`software_background`, `hardware_background`).
- **Backend (`./rag-chatbot`)**:
  - Framework: FastAPI (Python 3.12+).
  - AI: `openai-agents-sdk` + `openai-chatkit`.
  - Database: `asyncpg` (Neon Postgres) for Auth/User Data.

### 2. Feature Requirements (Non-Negotiable)
- **F1: Authentication**: Must implement a Signup/Signin flow that captures the user's technical background (e.g., "Python Developer", "Beginner").
- **F2: Personalization**: Every chapter page must have a **"Personalize"** button that rewrites content using the user's captured background.
- **F3: Localization**: Every chapter page must have a **"Translate to Urdu"** button powered by the AI backend.

### 3. Architecture Standards
- **UI Component**: Create a `ChapterToolbar` component injected at the top of every documentation page via Swizzling (`DocItem`).
- **Agent Skill**: Backend logic for personalization and translation MUST be encapsulated in reusable `Skill` classes (e.g., `src/skills/personalize.py`), NOT hardcoded in routes.

### 4. Design System ("Neo-Tech")
- All new UI elements (Buttons, Modals, Forms) must strictly follow the existing **Neo-Tech** theme:
  - Colors: Deep Navy (`#0f172a`) & Electric Purple (`#7c3aed`).
  - Style: Glassmorphism, Rounded Corners, Glowing Borders.

## Governance
This Constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified.

**Version**: 1.1.0 | **Ratified**: TODO(RATIFICATION_DATE): Set the original adoption date. | **Last Amended**: 2025-12-13