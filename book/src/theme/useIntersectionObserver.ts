import { useEffect, useState, useRef } from 'react';

export function useIntersectionObserver(options) {
  const [elements, setElements] = useState([]);
  const [entries, setEntries] = useState([]);

  const observer = useRef(null);

  useEffect(() => {
    if (elements.length === 0) return;

    observer.current = new IntersectionObserver((ioEntries) => {
      setEntries(ioEntries);
    }, options);

    elements.forEach(element => {
      observer.current.observe(element);
    });

    return () => {
      if (observer.current) {
        observer.current.disconnect();
      }
    };
  }, [elements, options]);

  return [observer.current, setElements, entries];
}

export function useIsVisible(elementRef) {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(([entry]) => {
      if (entry.isIntersecting) {
        setIsVisible(true);
        observer.unobserve(entry.target);
      }
    });

    if (elementRef.current) {
      observer.observe(elementRef.current);
    }

    return () => {
      if (observer) {
        observer.disconnect();
      }
    };
  }, [elementRef]);

  return isVisible;
}
