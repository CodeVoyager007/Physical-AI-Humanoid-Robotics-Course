from genai import Client
from settings import settings
import sys

client = Client(api_key=settings.GEMINI_API_KEY)

# List available models
print("Listing available models...")
try:
    for model in client.models.list():
        print(f"  - {model.name} (capabilities: {model.supported_generation_methods})")
except Exception as e:
    print(f"Error listing models: {e}")
    sys.exit(1)

# Test the configured model
print(f"\nTesting configured model: {settings.CHAT_MODEL}...")
try:
    response = client.models.generate_content(
        model=settings.CHAT_MODEL,
        contents="Say hello!"
    )
    print(f"✓ SUCCESS with {settings.CHAT_MODEL}")
    print(f"Response: {response.text}")
except Exception as e:
    print(f"✗ Error: {e}")

