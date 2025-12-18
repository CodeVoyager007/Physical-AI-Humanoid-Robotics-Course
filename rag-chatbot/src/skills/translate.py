import google.genai as genai
from settings import settings

class TranslationSkill:
    def __init__(self):
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model = genai.GenerativeModel(settings.CHAT_MODEL)

    def translate_to_urdu(self, text: str) -> str:
        system_prompt = (
            "You are an expert technical translator. Your task is to translate the given technical documentation into Urdu. "
            "It is crucial to maintain the original meaning and technical accuracy. "
            "Keep common technical terms like 'Node', 'Topic', 'ROS2', 'Python', 'C++' in English, as they are widely understood in that context. "
            "IMPORTANT: Provide ONLY the plain text translation, without any Markdown formatting (e.g., bold, italics, headers) or HTML tags."
        )
        
        response = self.model.generate_content([system_prompt, text])
        return response.text

if __name__ == '__main__':
    # Example Usage
    skill = TranslationSkill()
    sample_text = "ROS2 nodes communicate using topics. A node publishes messages to a topic, and other nodes subscribe to that topic to receive the messages. This is a fundamental concept in ROS2."
    
    translated_text = skill.translate_to_urdu(sample_text)
    print("--- Original Text ---")
    print(sample_text)
    print("\n--- Translated to Urdu ---")
    print(translated_text)
