from qdrant_client import QdrantClient
from qdrant_client.http import models
from settings import settings
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_qdrant_client() -> QdrantClient:
    url = settings.QDRANT_URL
    # Mask API key for logging if it's in the URL (unlikely but possible)
    logger.info(f"Initializing QdrantClient with URL: {url}")
    return QdrantClient(
        location=url, 
        api_key=settings.QDRANT_API_KEY,
        check_compatibility=False
    )

def init_db():
    try:
        client = get_qdrant_client()
        collection_name = "documents"
        
        logger.info(f"Checking if collection '{collection_name}' exists...")
        if not client.collection_exists(collection_name):
            logger.info(f"Collection '{collection_name}' not found. Creating...")
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Dimension for text-embedding-004
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Collection '{collection_name}' created.")
        else:
            logger.info(f"Collection '{collection_name}' already exists.")
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {e}")
        # Ensure the error is visible in logs
        raise e

if __name__ == "__main__":
    init_db()

import asyncpg
import os

class Database:
    def __init__(self):
        self.pool = None

    async def connect(self):
        self.pool = await asyncpg.create_pool(dsn=os.getenv("DATABASE_URL"))

    async def disconnect(self):
        if self.pool:
            await self.pool.close()

    async def fetch(self, query, *args):
        if not self.pool:
            await self.connect()
        async with self.pool.acquire() as connection:
            return await connection.fetch(query, *args)

    async def execute(self, query, *args):
        if not self.pool:
            await self.connect()
        async with self.pool.acquire() as connection:
            return await connection.execute(query, *args)

db = Database()
