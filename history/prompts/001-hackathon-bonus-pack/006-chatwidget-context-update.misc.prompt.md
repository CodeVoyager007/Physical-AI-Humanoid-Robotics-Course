---
id: "006"
title: "Update ChatWidget to support selected context like ChatGPT"
stage: "misc"
date: "2025-12-21"
surface: "agent"
model: "gemini-2.5-flash"
feature: "hackathon-bonus-pack"
branch: "main"
user: "user"
command: "no instead of showing in input field the selected text is should appear like when happens on chatgpt, and the chatbot uses that as context"
labels: ["ui", "chatbot", "chatgpt-style", "context"]
links: null
files:
 - "book/src/theme/ChatWidget.tsx"
 - "book/src/theme/ChatWidget.module.css"
tests: []
prompt: "no instead of showing in input field the selected text is should appear like when happens on chatgpt, and the chatbot uses that as context"
response: "Updated `ChatWidget.tsx` and its CSS to handle selected text as a distinct context block (ChatGPT style) instead of populating the input field. The input field is now cleared when context is added, and the context is displayed in a styled block above the input area. Form submission is allowed with context only."
---
Updated `ChatWidget` to improved context handling:
- **UI**: Styled `inputContext` to look like a quoted block (left border, muted background) above the input field.
- **UX**:
  - `ask-ai` event now clears the input field (preventing text from appearing there).
  - Placeholder updates to "Ask about the context..." when context is present.
  - Allows submitting the form if `selectedContext` is present, even if `input` is empty.
- **Backend**: Sends `context` separately from `query`.
