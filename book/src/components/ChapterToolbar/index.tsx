import React, { useState, useEffect, useRef } from 'react';
import styles from './ChapterToolbar.module.css';

const ChapterToolbar: React.FC = () => {
  const [isLoading, setIsLoading] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [isTranslated, setIsTranslated] = useState(false);
  // Ref to store the actual original DOM nodes of the MDX content
  const originalContentNodesRef = useRef<ChildNode[] | null>(null);

  const getMdxContentElement = () => {
    // Docusaurus renders markdown content inside a div with the 'markdown' class
    return document.querySelector('article .markdown') as HTMLElement;
  };

  // Effect to manage reset translation state if the page changes
  useEffect(() => {
    const mdxContent = getMdxContentElement();
    if (mdxContent && isTranslated) {
      // If we are on a new page and were translated, reset to original view
      if (originalContentNodesRef.current) {
        // Clear current content
        while (mdxContent.firstChild) {
          mdxContent.removeChild(mdxContent.firstChild);
        }
        // Restore original nodes
        originalContentNodesRef.current.forEach(node => mdxContent.appendChild(node));
      }
      originalContentNodesRef.current = null; // Clear saved nodes
      setIsTranslated(false);
    }
  }, [window.location.pathname]); // Dependency on pathname for page changes

  const handlePersonalize = async () => {
    setIsLoading(true);
    setPersonalizedContent(null);
    const mainContent = getMdxContentElement();
    if (!mainContent) {
      console.error('Could not find main article content to personalize.');
      setIsLoading(false);
      return;
    }

    try {
      const response = await fetch('http://localhost:8000/personalize', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: mainContent.innerText,
          software_background: 'Unknown',
          hardware_background: 'None',
        }),
      });
      const data = await response.json();
      if (response.ok) {
        setPersonalizedContent(data.personalized_text);
      } else {
        console.error('Personalization failed:', data);
      }
    } catch (error) {
      console.error('Network error during personalization:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleTranslate = async () => {
    const mdxContent = getMdxContentElement();
    if (!mdxContent) {
      console.error('Could not find main article content to translate.');
      return;
    }

    if (isTranslated) {
      // Revert to original content
      if (originalContentNodesRef.current) {
        // Clear current content
        while (mdxContent.firstChild) {
          mdxContent.removeChild(mdxContent.firstChild);
        }
        // Restore original nodes
        originalContentNodesRef.current.forEach(node => mdxContent.appendChild(node));
      }
      setIsTranslated(false);
      originalContentNodesRef.current = null; // Clear saved nodes after restoring
      return;
    }

    setIsLoading(true);
    // Save current child nodes before translation
    originalContentNodesRef.current = Array.from(mdxContent.childNodes);
    const textToTranslate = mdxContent.innerText; 

    try {
      const response = await fetch('http://localhost:8000/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: textToTranslate }),
      });
      const data = await response.json();
      if (response.ok) {
        // Clear original content
        while (mdxContent.firstChild) {
          mdxContent.removeChild(mdxContent.firstChild);
        }
        // Create a <pre> element for plain text display
        const preElement = document.createElement('pre');
        preElement.className = styles.translatedContentPre; // Apply styling
        preElement.textContent = data.translated_text; // Set plain text content
        mdxContent.appendChild(preElement); // Append to the article
        setIsTranslated(true);
      } else {
        console.error('Translation failed:', data);
        // If failed, clear saved original nodes to avoid restoring partial state
        originalContentNodesRef.current = null;
      }
    } catch (error) {
      console.error('Network error during translation:', error);
      originalContentNodesRef.current = null; // Don't keep original if failed
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      <div className={styles.toolbar}>
        <button className={styles.button} disabled={isLoading} onClick={handlePersonalize}>
          {isLoading ? 'Processing...' : '🪄 Personalize'}
        </button>
        <button className={styles.button} disabled={isLoading} onClick={handleTranslate}>
          {isTranslated ? 'Show Original' : '🌐 Urdu'}
        </button>
      </div>
      {/* Overlay for Personalized Content */}
      {personalizedContent && (
        <div className={styles.overlay}>
          <div className={styles.personalizedContent}>
            <button className={styles.closeButton} onClick={() => setPersonalizedContent(null)}>&times;</button>
            <h3>Personalized Content</h3>
            <div dangerouslySetInnerHTML={{ __html: personalizedContent.replace(/\n/g, '<br />') }} />
          </div>
        </div>
      )}
    </>
  );
};

export default ChapterToolbar;
