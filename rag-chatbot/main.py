from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager
from database import init_db
from rag_service import rag_service
from ingest import ingest_local_docs
from typing import Optional

from src.skills.personalize import PersonalizationSkill
from src.skills.translate import TranslationSkill

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Initialize DB on startup
    init_db()
    yield

app = FastAPI(lifespan=lifespan)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Docusaurus default port
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    query: str
    context: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

@app.get("/")
async def root():
    return {"message": "Hello World"}

@app.post("/ingest-local")
async def ingest_local(background_tasks: BackgroundTasks):
    try:
        background_tasks.add_task(ingest_local_docs)
        return {"status": "success", "message": "Local ingestion process started in the background."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/chat")
async def chat(request: ChatRequest):
    try:
        print(f"Received chat request: query='{request.query}', context='{request.context}'")
        answer = rag_service.generate_answer(
            request.query,
            request.context,
            request.software_background,
            request.hardware_background
        )
        return {"answer": answer}
    except Exception as e:
        import traceback
        print(traceback.format_exc())
        raise HTTPException(status_code=500, detail=str(e))

class PersonalizeRequest(BaseModel):
    text: str
    software_background: str
    hardware_background: str

personalization_skill = PersonalizationSkill()

@app.post("/personalize")
async def personalize_content(request: PersonalizeRequest):
    try:
        personalized_text = personalization_skill.personalize_content(
            request.text,
            request.software_background,
            request.hardware_background
        )
        return {"personalized_text": personalized_text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

class TranslateRequest(BaseModel):
    text: str
    target_lang: str = "urdu"

translation_skill = TranslationSkill()

@app.post("/translate")
async def translate_content(request: TranslateRequest):
    try:
        translated_text = translation_skill.translate_to_urdu(request.text)
        return {"translated_text": translated_text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
