---
id: 001
title: Create Project Constitution
stage: constitution
date: 2025-12-02
surface: agent
model: Gemini
feature: none
branch: master
user: aish-mughal\dell
command: /sp.constitution
labels: [constitution, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - "N/A"
---

## Prompt

/sp.constitution # Establish Project Constitution

**Objective**: Formalize the foundational principles of the "RoboLearn Handbook" project into a `constitution.md` document. This constitution will define the project's core laws regarding technology stack, design system, content structure, and future architecture, serving as the guiding document for all subsequent development.

1. **Project Definition**:
   - **Title**: "RoboLearn: Physical AI & Humanoid Robotics".
   - **Goal**: A comprehensive textbook teaching ROS 2, NVIDIA Isaac Sim, Gazebo, and VLA (Vision-Language-Action) Models.
   - **Audience**: University students and engineers moving from digital AI to embodied intelligence.

2. **Tech Stack Laws**:
   - **Framework**: Docusaurus (Latest).
   - **Language**: TypeScript (`.tsx`, `.ts`) ONLY. Strict typing is required.
   - **Styling**: Global CSS variables (`src/css/custom.css`).

3. **Design System ("Neo-Tech" Aesthetic)**:
   - **Vibe**: High-tech, clean, professional (inspired by Vercel/Stripe docs). NOT "Hacker/Matrix".
   - **Color Palette**:
     - **Background**: Deep Indigo/Navy (`#0f172a`).
     - **Text**: Slate White (`#e2e8f0`).
     - **Accents**: Electric Blue & Purple Gradients (to match the "Robotics/NVIDIA" vibe).
   - **UI Elements**: Glassmorphism (blur effects) on Sidebar/Navbar. Clean Sans-Serif typography.

4. **Content Structure**:
   - The content must rigidly follow the "Physical AI" syllabus: ROS 2, Simulation (Digital Twins), Isaac Platform, and VLA.
   - 30+ Chapters are required.

5. **Integration Readiness**:
   - The architecture must be prepared to accept a Context-Aware RAG Chatbot Widget in `src/theme/Root.js` (Wrapper pattern) in the next phase.

## Response snapshot

Constitution `v1.0.0` has been created. The principles for Tech Stack, Design, Content, and Integration Readiness are now codified. I have noted that dependent templates (`plan`, `spec`, `tasks`) may need future alignment.

Suggested commit message:
`docs: establish constitution v1.0.0`

## Outcome

- ✅ Impact: Established the foundational governance and principles for the project.
- 🧪 Tests: No tests were run as this was a documentation change.
- 📁 Files: Modified `.specify/memory/constitution.md`.
- 🔁 Next prompts: The user can now proceed with further project setup or content creation, following the established constitution.
- 🧠 Reflection: The process of creating the constitution directly from user input was smooth. The agent-native fallback for PHR creation worked as expected.

## Evaluation notes (flywheel)

- Failure modes observed: The initial shell command to create the PHR failed because the script was not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
