<!--
Sync Impact Report:
- Version change: 1.0.0 -> 1.0.0 (initialization)
- List of modified principles:
  - [PRINCIPLE_1_NAME] -> I. Tech Stack Laws
  - [PRINCIPLE_2_NAME] -> II. Design System ("Neo-Tech" Aesthetic)
  - [PRINCIPLE_3_NAME] -> III. Content Structure
  - [PRINCIPLE_4_NAME] -> IV. Integration Readiness
- Added sections: None
- Removed sections:
  - PRINCIPLE_5_NAME
  - PRINCIPLE_6_NAME
  - SECTION_2_NAME
  - SECTION_3_NAME
- Templates requiring updates:
  - ⚠ pending: .specify/templates/plan-template.md
  - ⚠ pending: .specify/templates/spec-template.md
  - ⚠ pending: .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# RoboLearn: Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Tech Stack Laws
- **Framework**: Docusaurus (Latest) MUST be used.
- **Language**: All code MUST be TypeScript (`.tsx`, `.ts`). Strict typing is non-negotiable.
- **Styling**: All styling MUST use global CSS variables defined in `src/css/custom.css`.

### II. Design System ("Neo-Tech" Aesthetic)
- **Aesthetic**: The user interface MUST adhere to a high-tech, clean, and professional "Neo-Tech" vibe, drawing inspiration from Vercel and Stripe documentation. It must NOT use a "Hacker/Matrix" theme.
- **Color Palette**:
  - **Background**: MUST be Deep Indigo/Navy (`#0f172a`).
  - **Text**: MUST be Slate White (`#e2e8f0`).
  - **Accents**: MUST use Electric Blue & Purple Gradients.
- **UI Elements**: The Sidebar and Navbar MUST implement Glassmorphism (blur effects). Typography MUST be a clean Sans-Serif font.

### III. Content Structure
- **Syllabus**: The content structure MUST rigidly follow the "Physical AI" syllabus, covering: ROS 2, Simulation (Digital Twins), the NVIDIA Isaac Platform, and Vision-Language-Action (VLA) models in that order.
- **Volume**: The textbook MUST contain 30 or more chapters.

### IV. Integration Readiness
- **Architecture**: The application architecture MUST be designed to accommodate a future Context-Aware RAG Chatbot Widget.
- **Implementation**: A wrapper pattern MUST be used, with the widget integrated into `src/theme/Root.js`.

## Governance
The project constitution is the single source of truth for development standards. All code, documentation, and architectural decisions must comply with these principles. Amendments require review and approval, with versioning to reflect the scope of change.

**Version**: 1.0.0 | **Ratified**: 2025-12-02 | **Last Amended**: 2025-12-02