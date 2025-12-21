---
id: "005"
title: "Fix process is not defined error in ChapterToolbar"
stage: "misc"
date: "2025-12-21"
surface: "agent"
model: "gemini-2.5-flash"
feature: "physical-ai-manual"
branch: "main"
user: "user"
command: "fix it"
labels: ["bugfix", "docusaurus", "client-side"]
links: null
files:
 - "book/docusaurus.config.ts"
 - "book/src/components/ChapterToolbar/index.tsx"
tests: []
prompt: "This page crashed. process is not defined fix it"
response: "Fixed the 'process is not defined' error by exposing the `DOCUSAURUS_TRANSLATE_API_URL` environment variable through `docusaurus.config.ts` customFields and updating `ChapterToolbar` to consume it via `useDocusaurusContext`. This prevents direct access to `process` in the browser environment."
---
Fixed the "process is not defined" error in the Docusaurus book.
- **Root Cause**: Direct usage of `process.env` in client-side React code (`ChapterToolbar`), which is not polyfilled in Webpack 5.
- **Fix**:
  1.  Added `customFields` to `book/docusaurus.config.ts` to pass `process.env.DOCUSAURUS_TRANSLATE_API_URL`.
  2.  Updated `book/src/components/ChapterToolbar/index.tsx` to use `useDocusaurusContext` to access the config value.
