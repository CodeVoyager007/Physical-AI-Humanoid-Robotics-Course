---
id: 004
title: Overhaul UI with RoboLearn Theme
stage: refactor
date: 2025-12-02
surface: agent
model: Gemini
feature: 001-physical-ai-manual
branch: 001-physical-ai-manual
user: aish-mughal\dell
command: /sp.implement
labels: [ui, theme, refactor, docusaurus]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - src/css/custom.css
  - src/pages/index.tsx
  - src/pages/index.module.css
  - docusaurus.config.ts
tests:
  - "npm run build"
---

## Prompt

/sp.implement # REFACTOR: Align UI with "Neo-Tech" Design System

**Objective**: Execute a UI refactoring phase to implement the high-fidelity "Neo-Tech" Design System. This involves a complete overhaul of the CSS and landing page to introduce advanced animations, glassmorphism, and a professional, "expensive" aesthetic as defined in the project's visual identity specification.

## 1. CSS Overhaul (`src/css/custom.css`)
Rewrite the entire CSS file. Do NOT use the default Docusaurus styles. Implement:
-   **Variables**:
    -   Background: `#0b0e14` (Deepest Navy).
    -   Surface: `#151923` (Card Background).
    -   Primary: `#3b82f6` (Electric Blue) to `#8b5cf6` (Violet) gradients.
-   **Global Animations**:
    -   Add `@keyframes fade-in-up` for landing page elements.
    -   Add transition properties (`transition: all 0.2s ease-in-out`) to ALL interactive elements (buttons, cards, links).
-   **Glassmorphism**: Navbar and Sidebar must have `backdrop-filter: blur(16px)` and `background: rgba(11, 14, 20, 0.8)`.
-   **Card Styling**: Feature cards must have a thin, glowing border on hover and slightly lift up (`transform: translateY(-5px)`).

## 2. Landing Page Upgrade (`src/pages/index.tsx`)
Refactor the landing page to look professional:
-   **Hero Section**: Use a large, centered Typography with a gradient text effect (`background-clip: text`).
-   **Animation**: Wrap the Hero title and buttons in a container that triggers the `fade-in-up` animation on load.
-   **Layout**: Use a masonry-style grid for the Chapter Modules.
-   **Buttons**: Make the "Start Learning" button a large, glowing gradient pill shape.

## 3. Sidebar & Typography
-   **Sidebar**: Remove the default gray border. Use spacing and hover-glow effects for links.
-   **Fonts**: Force `Inter` or `SF Pro Display`. Increase line-height for better readability.

**Constraint**: Make it look expensive. No default Docusaurus styling allowed.

## Response snapshot

UI Overhaul complete.

- The theme defined in `src/css/custom.css` has been completely rewritten to match the "RoboLearn" high-fidelity design.
- The landing page at `src/pages/index.tsx` has been refactored with a new hero section, animations, and a professional layout.
- The build was successful, and the `onBrokenLinks` setting was reverted to `'throw'`.
- All changes have been committed.

## Outcome

- ✅ **Impact**: Transformed the site's visual identity from the Docusaurus default to a polished, professional, and "expensive" looking "Neo-Tech" theme.
- 🧪 **Tests**: The `npm run build` command was run successfully, verifying that the changes did not break the site structure.
- 📁 **Files**:
  - `src/css/custom.css` (Overwritten)
  - `src/pages/index.tsx` (Overwritten)
  - `src/pages/index.module.css` (Created)
  - `docusaurus.config.ts` (Modified and reverted)
- 🔁 **Next prompts**: The project is now in a visually complete state.
- 🧠 **Reflection**: The direct implementation of a new design spec was successful, demonstrating the agility of the Spec-Driven Development (SDD) approach to accommodate refactoring and UI overhauls based on a clear, high-level specification. The process involved direct file modifications based on a design brief, followed by verification.

## Evaluation notes (flywheel)

- **Failure modes observed**: None during this specific overhaul. The build issues from the previous turn informed the process.
- **Graders run and results (PASS/FAIL)**: N/A
- **Prompt variant (if applicable)**: N/A
- **Next experiment (smallest change to try)**: N/A
