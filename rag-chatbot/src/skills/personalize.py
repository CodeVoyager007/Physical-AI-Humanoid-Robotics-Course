import google.genai as genai
from settings import settings

class PersonalizationSkill:
    def __init__(self):
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model = genai.GenerativeModel(settings.CHAT_MODEL)

    def personalize_content(self, text: str, software_background: str, hardware_background: str) -> str:
        system_prompt = (
            f"You are an expert technical content rewriter. Your goal is to adjust the provided technical content "
            f"for a user with a '{software_background}' software background and a '{hardware_background}' hardware background. "
            f"Adjust analogies, complexity, and examples to best suit this user's expertise. "
            f"Maintain the core technical accuracy and meaning."
        )
        
        # Consider using a more robust chat approach for better control over system instructions
        # For simplicity, directly using generate_content with a combined prompt
        response = self.model.generate_content([system_prompt, text])
        return response.text

if __name__ == '__main__':
    # Example Usage
    skill = PersonalizationSkill()
    sample_text = "ROS2 nodes communicate using topics. A node publishes messages to a topic, and other nodes subscribe to that topic to receive the messages."
    
    # Example 1: Python Developer, Arduino
    personalized_text_1 = skill.personalize_content(sample_text, "Python Dev", "Arduino")
    print("--- Python Dev, Arduino ---")
    print(personalized_text_1)
    
    # Example 2: Beginner, None
    personalized_text_2 = skill.personalize_content(sample_text, "Beginner", "None")
    print("\n--- Beginner, None ---")
    print(personalized_text_2)
