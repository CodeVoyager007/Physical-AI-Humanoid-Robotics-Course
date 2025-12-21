from typing import Optional
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
from settings import settings
from database import get_qdrant_client
import uuid
import os

class RAGService:
    def __init__(self):
        # OpenAI SDK for embeddings and chat
        self.openai_client = OpenAI(
            api_key=settings.GEMINI_API_KEY,
            base_url=settings.GEMINI_BASE_URL
        )
        self.qdrant_client = get_qdrant_client()
        self.collection_name = "documents"

    def generate_embedding(self, text: str) -> list[float]:
        response = self.openai_client.embeddings.create(
            model=settings.EMBEDDING_MODEL,
            input=text
        )
        return response.data[0].embedding

    def upsert_document_from_file(self, file_path: str):
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
        
        # Chunk the content
        chunk_size = 500
        chunks = [content[i:i+chunk_size] for i in range(0, len(content), chunk_size)]
        
        points = []
        for chunk in chunks:
            embedding = self.generate_embedding(chunk)
            point_id = str(uuid.uuid4())
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={"text": chunk, "filename": os.path.basename(file_path)}
                )
            )
        
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        return len(points)

    def search(self, query: str, limit: int = 3) -> list[dict]:
        query_embedding = self.generate_embedding(query)
        
        search_result = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=limit
        )
        
        return [hit.payload for hit in search_result.points]


    def generate_answer(self, query: str, context: str = None, software_background: Optional[str] = None, hardware_background: Optional[str] = None) -> dict:
        # Step 1: Synthesize a factual answer from the context.
        search_query = context if context else query
        context_payloads = self.search(search_query)

        retrieved_context_str = "\n\n".join([f"Source: {p.get('filename', 'N/A')}\nContent: {p.get('text', '')}" for p in context_payloads])

        synthesis_context = ""
        if context:
            synthesis_context += f"User provided context:\n{context}\n\n"
        if retrieved_context_str:
            synthesis_context += f"Retrieved from book:\n{retrieved_context_str}"

        if not synthesis_context.strip():
            synthesis_context = "No information was found."

        synthesis_prompt = f"""Synthesize the following information into a concise, factual summary that directly answers the user's question.

Information:
---
{synthesis_context}
---

User's Question: {query}

Factual Summary:"""
        
        synthesis_response = self.openai_client.chat.completions.create(
            model=settings.CHAT_MODEL,
            messages=[
                {"role": "system", "content": "You are a helpful assistant that synthesizes information."},
                {"role": "user", "content": synthesis_prompt}
            ]
        )
        factual_summary = synthesis_response.choices[0].message.content

        # Step 2: Convert the factual summary into a conversational response.
        # Updated persona to match customer service style (GoDaddy, Amazon)
        persona_system_instruction = (
            "You are a polite, helpful, and efficient customer service assistant, similar to those found on platforms like GoDaddy or Amazon. "
            "Your personality is professional, proactive, and friendly. You specialize in the content of the book \"A Textbook for Teaching Physical AI & Humanoid Robotics\"."
            " You must be CONCISE and answer ONLY what is asked. Do not volunteer extra information unless it is critical for safety or understanding."
        )

        if software_background and hardware_background:
            persona_system_instruction += f" You are assisting a customer with a '{software_background}' software background and a '{hardware_background}' hardware background. Tailor your support to their expertise level."

        persona_prompt = f"""{persona_system_instruction}

Your goal is to rephrase the following "Factual Summary" into a professional customer service response for the "Original User Question".

**CRITICAL INSTRUCTIONS:**
- NEVER mention that you are using a summary or information provided to you.
- Provide a direct, helpful answer. Use phrases like "I'd be happy to help with that" or "Certainly!" where appropriate, matching the vibe of a top-tier support agent.
- If the information is not available, politely inform the user and offer to assist with something else.

Original User Question: {query}

Factual Summary:
---
{factual_summary}
---

Your Professional Support Answer:"""

        final_response = self.openai_client.chat.completions.create(
            model=settings.CHAT_MODEL,
            messages=[
                {"role": "system", "content": persona_system_instruction},
                {"role": "user", "content": persona_prompt}
            ]
        )
        
        answer = final_response.choices[0].message.content
        
        # Prepare "attachments" (sources)
        attachments = []
        for p in context_payloads:
            attachments.append({
                "filename": p.get("filename", "unknown"),
                "content": p.get("text", "")
            })

        return {
            "answer": answer,
            "attachments": attachments
        }

rag_service = RAGService()
