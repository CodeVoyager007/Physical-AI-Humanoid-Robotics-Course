# Quickstart: Hackathon Bonus Pack (Revised)

## Backend Setup

1.  Navigate to the `rag-chatbot` directory.
2.  Install dependencies: `pip install -r requirements.txt` (or `pip install -e .` if using `pyproject.toml`)
3.  Run the server: `uvicorn src.main:app --reload`

The backend will be running at `http://localhost:8000`.

## Frontend Setup

1.  Navigate to the `book` directory.
2.  Install dependencies: `npm install`
3.  Run the development server: `npm start`

The frontend will be running at `http://localhost:3000`.

## Usage

1.  Open the frontend and navigate to any chapter page.
2.  Click the "Personalize" button to see the content tailored to a generic "Unknown" profile.
3.  Click the "Translate to Urdu" button to see the content in Urdu. Click "Show Original" to revert.
4.  Open the chat widget and select your software and hardware background from the dropdowns to get customized responses.
