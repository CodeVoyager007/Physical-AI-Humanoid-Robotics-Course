import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';
import { useIsVisible } from './useIntersectionObserver'; // Corrected import for named export

// Define a type for individual messages
type Attachment = {
  filename: string;
  content: string;
};

type Message = {
  text: string;
  sender: 'user' | 'bot';
  isTyping?: boolean;
  attachments?: Attachment[];
};

// SVG Icon for the send button
const SendIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2 .01 7z" fill="currentColor"/>
  </svg>
);

// Main Chat Widget Component
export default function ChatWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [selectedContext, setSelectedContext] = useState<string | null>(null); // New state for selected context
  const [softwareBackground, setSoftwareBackground] = useState('Beginner'); // New state
  const [hardwareBackground, setHardwareBackground] = useState('None');     // New state
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Use useIsVisible for observing animation
  const widgetRef = useRef<HTMLDivElement>(null);
  const isVisible = useIsVisible(widgetRef); // Using useIsVisible directly

  // Automatically scroll to the latest message
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);
  
  // Handle the 'ask-ai' event
  useEffect(() => {
    const handleAskAi = (event) => {
      setIsOpen(true);
      setSelectedContext(event.detail); // Populate selectedContext with selected text
      setInput(''); // Ensure input is clear so it doesn't look like it's in the input field
      inputRef.current?.focus();
    };

    window.addEventListener('ask-ai', handleAskAi);
    return () => {
      window.removeEventListener('ask-ai', handleAskAi);
    };
  }, []);

  // Show a welcome message when the chat is opened
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      setMessages([
        { text: 'Hello! How can I help you today?', sender: 'bot' }
      ]);
    }
  }, [isOpen]);

  // Handle input changes
  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setInput(e.target.value);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && e.ctrlKey) {
      e.preventDefault();
      const { selectionStart, selectionEnd, value } = e.currentTarget;
      const newValue = value.substring(0, selectionStart) + '\n' + value.substring(selectionEnd);
      setInput(newValue);
      // Manually set cursor position after updating state
      // This requires a small delay to ensure state update is processed
      setTimeout(() => {
        if (inputRef.current) {
          inputRef.current.selectionStart = selectionStart + 1;
          inputRef.current.selectionEnd = selectionStart + 1;
        }
      }, 0);
    } else if (e.key === 'Enter') {
      e.preventDefault(); // Prevent default Enter behavior (e.g., new line in some browsers)
      handleSubmit(e as any); // Submit the form if only Enter is pressed
    }
  };

  // Handle form submission
  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    // Allow submit if there is context, even if input is empty.
    if (!input.trim() && !selectedContext) return;

    const userMessage: Message = { 
      text: input, 
      sender: 'user',
      attachments: selectedContext ? [{ filename: 'Selected Context', content: selectedContext }] : undefined
    };

    setMessages((prev) => [...prev, userMessage, { text: '', sender: 'bot', isTyping: true }]);
    const currentContext = selectedContext; // Capture current context
    const currentQuery = input;
    setInput('');
    setSelectedContext(null); // Clear context after sending

    try {
      const response = await fetch('https://physical-ai-humanoid-robotics-course-1-b9c1.onrender.com/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: currentQuery || "Context provided", # Send a default query if input is empty but context exists
          context: currentContext, // Send selected text as context
          software_background: softwareBackground,
          hardware_background: hardwareBackground,
        }),
      });

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      
      const data = await response.json();
      const botMessage: Message = { 
        text: data.answer, 
        sender: 'bot',
        attachments: data.attachments // Add attachments from response
      };
      setMessages((prev) => [...prev.filter(m => !m.isTyping), botMessage]);

    } catch (error) {
      console.error('Error fetching chatbot response:', error);
      const errorMessage: Message = { text: 'Sorry, something went wrong. Please try again.', sender: 'bot' };
      setMessages((prev) => [...prev.filter(m => !m.isTyping), errorMessage]);
    }
  };

  return (
    <div ref={widgetRef} className={`${styles.widgetContainer} ${isVisible ? styles.visible : ''}`}>
      {/* Floating Action Button */}
      {!isOpen && (
        <button className={styles.fab} onClick={() => setIsOpen(true)} aria-label="Open chat">
          <span className={styles.fabIcon}>💬</span>
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3 className={styles.headerTitle}>AI Assistant</h3>
            <button className={styles.closeButton} onClick={() => setIsOpen(false)} aria-label="Close chat">
              &times;
            </button>
          </div>

          {/* Background Selection Dropdowns */}
          <div className={styles.backgroundSelectors}>
            <div className={styles.selectorGroup}>
              <label htmlFor="softwareBackground">Software:</label>
              <select
                id="softwareBackground"
                value={softwareBackground}
                onChange={(e) => setSoftwareBackground(e.target.value)}
                className={styles.selectField}
              >
                <option value="Beginner">Beginner</option>
                <option value="Python Dev">Python Dev</option>
                <option value="C++ Dev">C++ Dev</option>
              </select>
            </div>
            <div className={styles.selectorGroup}>
              <label htmlFor="hardwareBackground">Hardware:</label>
              <select
                id="hardwareBackground"
                value={hardwareBackground}
                onChange={(e) => setHardwareBackground(e.target.value)}
                className={styles.selectField}
              >
                <option value="None">None</option>
                <option value="Arduino">Arduino</option>
                <option value="Jetson">Jetson</option>
              </select>
            </div>
          </div>
          {/* End Background Selection Dropdowns */}

          {/* Messages Area */}
          <div className={styles.messagesArea}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                {msg.isTyping ? (
                  <div className={styles.typingIndicator}><span></span><span></span><span></span></div>
                ) : (
                  <>
                    {msg.text}
                    {msg.attachments && msg.attachments.length > 0 && (
                      <div className={styles.attachmentsList}>
                        {msg.attachments.map((att, i) => (
                          <div key={i} className={styles.attachmentItem} title={att.content}>
                            <span className={styles.attachmentIcon}>📎</span>
                            <span className={styles.attachmentName}>{att.filename}</span>
                          </div>
                        ))}
                      </div>
                    )}
                  </>
                )}
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form onSubmit={handleSubmit} className={styles.inputForm}>
            {selectedContext && (
              <div className={styles.inputContext}>
                <div className={styles.inputContextText} title={selectedContext}>
                  {selectedContext}
                </div>
                <button 
                  type="button" 
                  className={styles.removeContextButton}
                  onClick={() => setSelectedContext(null)}
                  aria-label="Remove context"
                >
                  &times;
                </button>
              </div>
            )}
            <div className={styles.inputContainer}>
              <textarea
                ref={inputRef}
                rows={1}
                className={styles.inputField}
                placeholder={selectedContext ? "Ask about the context..." : "Ask me anything..."}
                value={input}
                onInput={handleInputChange} // Use onInput for textarea
                onKeyDown={handleKeyDown} // Add onKeyDown handler
                aria-label="Chat input"
              ></textarea>
              <button type="submit" className={styles.sendButton} aria-label="Send message">
                <SendIcon />
              </button>
            </div>
          </form>
        </div>
      )}
    </div>
  );
}