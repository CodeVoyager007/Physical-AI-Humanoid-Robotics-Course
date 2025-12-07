import React, { useState, useEffect, useCallback } from 'react';
import ChatWidget from './ChatWidget';
import SelectionToolbar from './SelectionToolbar';

export default function Root({ children }) {
  const [selection, setSelection] = useState(null);
  const [toolbarPosition, setToolbarPosition] = useState({ x: 0, y: 0 });
  const [showToolbar, setShowToolbar] = useState(false);

  const handleSelection = useCallback(() => {
    const currentSelection = window.getSelection();
    if (currentSelection && currentSelection.toString().trim().length > 0) {
      const range = currentSelection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      setSelection(currentSelection.toString());
      setToolbarPosition({
        x: rect.left + window.scrollX + rect.width / 2,
        y: rect.top + window.scrollY - 40,
      });
      setShowToolbar(true);
    } else {
      setShowToolbar(false);
    }
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, [handleSelection]);

  const handleAskAi = () => {
    // This is a bit of a hack, but it's the easiest way to communicate with the ChatWidget
    // without a complex state management solution.
    const event = new CustomEvent('ask-ai', { detail: selection });
    window.dispatchEvent(event);
    setShowToolbar(false);
  };

  const handleCopy = () => {
    navigator.clipboard.writeText(selection);
    setShowToolbar(false);
  };

  return (
    <>
      {children}
      <ChatWidget />
      {showToolbar && (
        <SelectionToolbar
          x={toolbarPosition.x}
          y={toolbarPosition.y}
          onAskAi={handleAskAi}
          onCopy={handleCopy}
        />
      )}
    </>
  );
}
