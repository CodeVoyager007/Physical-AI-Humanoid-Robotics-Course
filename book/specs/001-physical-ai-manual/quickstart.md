# Quickstart: RoboLearn Development

This guide provides the essential steps to set up and run the RoboLearn documentation site locally.

## Prerequisites

-   Node.js (LTS version)
-   npm (comes with Node.js)

## Setup

1.  **Clone the Repository**:
    ```bash
    git clone <repository-url>
    cd book
    ```

2.  **Install Dependencies**:
    Install all the necessary packages defined in `package.json`.
    ```bash
    npm install
    ```

## Running the Development Server

To start the local development server with hot-reloading:

```bash
npm start
```

The site will be available at `http://localhost:3000`.

## Building the Static Site

To create a production-ready static build of the website:

```bash
npm run build
```

The output will be generated in the `build/` directory. This is the folder that would be deployed to a web server.
