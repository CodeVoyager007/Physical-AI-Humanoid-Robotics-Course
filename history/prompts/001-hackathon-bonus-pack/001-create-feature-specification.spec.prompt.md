---
id: 001
title: Create Feature Specification for Hackathon Bonus Pack
stage: spec
date: 2025-12-13
surface: agent
model: gemini
feature: 001-hackathon-bonus-pack
branch: 001-hackathon-bonus-pack
user: user
command: /sp.specify # Feature Specification: Hackathon Bonus Pack...
labels: [spec, hackathon, bonus-pack]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/001-hackathon-bonus-pack/spec.md
  - specs/001-hackathon-bonus-pack/checklists/requirements.md
tests_yaml: |
  - "No tests run"
prompt_text: |
  # Feature Specification: Hackathon Bonus Pack

  **Objective:** Implement Authentication, Personalization, and Localization features for the "Sentient Machines" platform.

  ## 1. Authentication System (Better-Auth)
  **Goal:** Capture user expertise to enable personalization.
  - **Frontend (`book/src/components/Auth`)**:
    - **`Software_Background` (Select: Beginner, Python Dev, C++ Dev), `Hardware_Background` (Select: None, Arduino, Jetson).**

  ## 2. Smart Chapter Toolbar
  **Goal:** Allow users to modify content on the fly.
  - **Component**: `book/src/components/ChapterToolbar/index.tsx`.
  - **Placement**: Injected at the top of every Markdown chapter.
  - **UI Elements**:
    - **Button 1 [🪄 Personalize]**: Active only if logged in. Sends chapter text + user profile to backend.
    - **Button 2 [🌐 Urdu]**: Active for all users. Sends chapter text to backend.
  - **Behavior**: When clicked, show a "Processing..." skeleton loader, then replace the chapter text (or show an overlay) with the AI-generated version.

  ## 3. Backend Intelligence (`rag-chatbot`)
  - **New Endpoints (`src/main.py`)**:
    - `POST /personalize`: Input `{ text, user_profile }`. Returns `{ personalized_text }`.
    - `POST /translate`: Input `{ text, target_lang="urdu" }`. Returns `{ translated_text }`.
  - **New Skills (`src/skills/`)**:
    - **`PersonalizationSkill`**: Uses Gemini 1.5 Flash. System Prompt: "Rewrite this technical content for a {user_profile}. Adjust analogies and complexity."
    - **`TranslationSkill`**: Uses Gemini 1.5 Flash. System Prompt: "Translate this technical documentation to Urdu. Maintain technical terms like 'Node' or 'Topic' in English where appropriate."

  ## 4. Integration Logic
  - **Data Flow**:
    1. User clicks button on Frontend.
    2. Frontend sends payload to `http://localhost:8000/...`.
    3. Backend Agent processes request.
    4. Frontend updates view with new text.

  do everything without doing any git work
response_text: |
  Created the feature specification and requirements checklist for the 'Hackathon Bonus Pack' feature.
outcome: "✅ Impact: The feature is now clearly defined and ready for the planning phase."
evaluation: |
  - Failure modes observed: None
  - Graders run and results (PASS/FAIL): PASS
  - Prompt variant (if applicable): None
  - Next experiment (smallest change to try): n/a
---
