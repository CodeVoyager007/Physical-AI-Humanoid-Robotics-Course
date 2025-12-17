import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';
import { useIsVisible } from './useIntersectionObserver'; // Corrected import for named export

// Define a type for individual messages
type Message = {
  text: string;
  sender: 'user' | 'bot';
  isTyping?: boolean;
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
      setInput(event.detail); // Populate input with selected text
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
  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInput(e.target.value);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
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
    if (!input.trim()) return;

    const userMessage: Message = { text: input, sender: 'user' };
    setMessages((prev) => [...prev, userMessage, { text: '', sender: 'bot', isTyping: true }]);
    setInput('');

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: input,
          software_background: softwareBackground, // Send software background
          hardware_background: hardwareBackground, // Send hardware background
        }),
      });

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      
      const data = await response.json();
      const botMessage: Message = { text: data.answer, sender: 'bot' }; // Changed from data.response to data.answer based on backend
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
                {msg.isTyping ? <div className={styles.typingIndicator}><span></span><span></span><span></span></div> : msg.text}
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <textarea
              ref={inputRef}
              rows={1}
              className={styles.inputField}
              placeholder="Ask me anything..."
              value={input}
              onInput={handleInputChange} // Use onInput for textarea
              onKeyDown={handleKeyDown} // Add onKeyDown handler
              aria-label="Chat input"
            ></textarea>
            <button type="submit" className={styles.sendButton} aria-label="Send message">
              <SendIcon />
            </button>
          </form>
        </div>
      )}
    </div>
  );
}