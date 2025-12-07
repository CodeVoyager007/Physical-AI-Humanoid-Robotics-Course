import React from 'react';
import styles from './SelectionToolbar.module.css';

type SelectionToolbarProps = {
  x: number;
  y: number;
  onAskAi: () => void;
  onCopy: () => void;
};

export default function SelectionToolbar({ x, y, onAskAi, onCopy }: SelectionToolbarProps) {
  return (
    <div className={styles.toolbar} style={{ left: x, top: y }}>
      <button onClick={onAskAi}>Ask AI</button>
      <button onClick={onCopy}>Copy</button>
    </div>
  );
}
