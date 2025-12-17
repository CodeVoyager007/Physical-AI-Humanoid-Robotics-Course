---
id: 001
title: Amend constitution for Sentient Machines Bonus Features
stage: constitution
date: 2025-12-13
surface: agent
model: gemini
feature: none
branch: master
user: user
command: /sp.constitution Constitution: Sentient Machines (Bonus Features)...
labels: [constitution, sentient-machines, hackathon]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - "No tests run"
---

## Prompt

/sp.constitution  Constitution: Sentient Machines (Bonus Features)

**Context:** Extending the existing "Sentient Machines" platform with Hackathon Bonus Features.
**Scope:** Covers both `book/` (Frontend) and `rag-chatbot/` (Backend).

## 1. Tech Stack Integration Laws
- **Frontend (`./book`)**:
  - Framework: Docusaurus v3 (TypeScript).
  - Auth: **Better-Auth** (Client-side integration).
  - State: React Context for managing User Profile (`software_background`, `hardware_background`).
- **Backend (`./rag-chatbot`)**:
  - Framework: FastAPI (Python 3.12+).
  - AI: `openai-agents-sdk` + `openai-chatkit`.
  - Database: `asyncpg` (Neon Postgres) for Auth/User Data.

## 2. Feature Requirements (Non-Negotiable)
- **F1: Authentication**: Must implement a Signup/Signin flow that captures the user's technical background (e.g., "Python Developer", "Beginner").
- **F2: Personalization**: Every chapter page must have a **"Personalize"** button that rewrites content using the user's captured background.
- **F3: Localization**: Every chapter page must have a **"Translate to Urdu"** button powered by the AI backend.

## 3. Architecture Standards
- **UI Component**: Create a `ChapterToolbar` component injected at the top of every documentation page via Swizzling (`DocItem`).
- **Agent Skill**: Backend logic for personalization and translation MUST be encapsulated in reusable `Skill` classes (e.g., `src/skills/personalize.py`), NOT hardcoded in routes.

## 4. Design System ("Neo-Tech")
- All new UI elements (Buttons, Modals, Forms) must strictly follow the existing **Neo-Tech** theme:
  - Colors: Deep Navy (`#0f172a`) & Electric Purple (`#7c3aed`).
  - Style: Glassmorphism, Rounded Corners, Glowing Borders.

## Response snapshot

Updated the project constitution to version 1.1.0, incorporating new principles for Tech Stack, Feature Requirements, Architecture, and Design System. Also updated the plan template to reflect these changes.

## Outcome

- ✅ Impact: The project constitution is now aligned with the new feature requirements.
- 🧪 Tests: No tests were run as this was a documentation change.
- 📁 Files: 2 files were modified.
- 🔁 Next prompts: The user can now proceed with planning and implementation based on the updated constitution.
- 🧠 Reflection: The process of updating the constitution and propagating changes to templates worked as expected.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): n/a
