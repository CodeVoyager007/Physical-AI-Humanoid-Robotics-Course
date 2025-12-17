# Data Model: RoboLearn Content

This document outlines the conceptual data model for the RoboLearn educational site. As a documentation-heavy project, the primary entities are related to the content structure itself.

## Core Entities

### 1. Part

Represents a major section of the textbook.

-   **Fields**:
    -   `title` (string): The name of the part (e.g., "The Awakening (Foundations)").
    -   `order` (integer): The numerical order in which the part appears.
-   **Representation**: A sub-directory within the `/docs` folder (e.g., `/docs/01-foundations`).

### 2. Chapter

Represents an individual chapter within a Part.

-   **Fields**:
    -   `title` (string): The title of the chapter, derived from the file's frontmatter or first H1 heading.
    -   `content` (Markdown/MDX): The full technical content of the chapter.
    -   `order` (integer): The numerical order within its Part.
-   **Representation**: A `.md` or `.mdx` file within a Part's directory (e.g., `/docs/01-foundations/01-embodied-intelligence.md`).

## Relationships

-   A **Part** contains one or more **Chapters**.
-   The entire collection of **Parts** and their **Chapters** forms the **Syllabus**.

This model is realized through the file system structure, which Docusaurus uses to build the site and generate navigation. No database is required.
