---
id: "007"
title: "Refine chatbot system prompt for conciseness"
stage: "misc"
date: "2025-12-21"
surface: "agent"
model: "gemini-2.5-flash"
feature: "rag-chatbot"
branch: "main"
user: "user"
command: "no it is not mandatory to ask by using context the chatbot should answer other questions too...and it replies with so much unasked things so fix that"
labels: ["prompt-engineering", "chatbot", "conciseness"]
links: null
files:
 - "rag-chatbot/rag_service.py"
tests: []
prompt: "no it is not mandatory to ask by using context the chatbot should answer other questions too...and it replies with so much unasked things so fix that"
response: "Updated `rag_service.py` to refine the `persona_system_instruction`. Added explicit instructions to be 'CONCISE and answer ONLY what is asked' and to avoid volunteering extra information. Verified that `ChatWidget.tsx` already supports asking questions without context."
---
Refined the chatbot's system prompt to address user feedback about verbosity:
- **Conciseness**: Added strict instruction: "You must be CONCISE and answer ONLY what is asked."
- **Relevance**: Explicitly forbade volunteering extra information unless critical.
- **Verification**: Confirmed `ChatWidget.tsx` submission logic (`if (!input.trim() && !selectedContext) return;`) correctly allows questions without context as long as there is input.
