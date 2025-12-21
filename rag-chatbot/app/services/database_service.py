"""
Neon PostgreSQL Database Service
Handles session management and conversation history
"""

import asyncpg
from typing import Optional, List, Dict, Any
from datetime import datetime
import json
import uuid

from ..config import get_settings


class DatabaseService:
    """Service for PostgreSQL database operations using Neon"""
    
    def __init__(self):
        settings = get_settings()
        self.database_url = settings.neon_database_url
        self.pool: Optional[asyncpg.Pool] = None
        # In-memory fallback when no remote DB configured (development)
        self._local_sessions: Dict[str, Dict[str, Any]] = {}
        self._local_history: Dict[str, List[Dict[str, Any]]] = {}
    
    async def connect(self):
        """Initialize connection pool"""
        # If no database URL configured, operate in local in-memory mode
        if not self.database_url:
            return

        if self.pool is None:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
    
    async def disconnect(self):
        """Close connection pool"""
        if self.pool:
            await self.pool.close()
            self.pool = None
    
    async def initialize_tables(self):
        """Create required tables if they don't exist"""
        # Skip initialization when running in-memory
        if not self.database_url:
            print("⚠️ Neon database URL not configured — using in-memory storage")
            return

        await self.connect()
        
        async with self.pool.acquire() as conn:
            # Sessions table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                    metadata JSONB DEFAULT '{}'::jsonb
                )
            """)
            
            # Conversation history table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS conversation_history (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
                    role VARCHAR(20) NOT NULL,
                    content TEXT NOT NULL,
                    sources JSONB DEFAULT '[]'::jsonb,
                    mode VARCHAR(50) DEFAULT 'full_book',
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
                )
            """)
            
            # Create indexes
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_history_session 
                ON conversation_history(session_id)
            """)
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_history_created 
                ON conversation_history(created_at DESC)
            """)
            
            # Feedback table for improving RAG
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_feedback (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    session_id UUID REFERENCES chat_sessions(id),
                    message_id UUID,
                    rating INTEGER CHECK (rating >= 1 AND rating <= 5),
                    feedback_text TEXT,
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
                )
            """)
    
    async def create_session(self, metadata: Dict[str, Any] = None) -> str:
        """Create a new chat session"""
        # In-memory fallback when DB not configured
        if not self.database_url:
            session_id = str(uuid.uuid4())
            self._local_sessions[session_id] = {
                "created_at": datetime.utcnow().isoformat(),
                "metadata": metadata or {}
            }
            self._local_history[session_id] = []
            return session_id

        await self.connect()

        session_id = str(uuid.uuid4())
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO chat_sessions (id, metadata) 
                VALUES ($1, $2)
                """,
                uuid.UUID(session_id),
                json.dumps(metadata or {})
            )

        return session_id
    
    async def get_or_create_session(self, session_id: Optional[str]) -> str:
        """Get existing session or create new one"""
        # In-memory mode
        if not self.database_url:
            if session_id and session_id in self._local_sessions:
                return session_id
            return await self.create_session()

        if session_id:
            # Verify session exists
            await self.connect()
            async with self.pool.acquire() as conn:
                result = await conn.fetchval(
                    "SELECT id FROM chat_sessions WHERE id = $1",
                    uuid.UUID(session_id)
                )
                if result:
                    # Update last activity
                    await conn.execute(
                        """
                        UPDATE chat_sessions 
                        SET updated_at = NOW() 
                        WHERE id = $1
                        """,
                        uuid.UUID(session_id)
                    )
                    return session_id

        # Create new session
        return await self.create_session()
    
    async def add_message(
        self,
        session_id: str,
        role: str,
        content: str,
        sources: List[Dict] = None,
        mode: str = "full_book"
    ) -> str:
        """Add a message to conversation history"""
        # In-memory fallback
        if not self.database_url:
            message_id = str(uuid.uuid4())
            entry = {
                "id": message_id,
                "role": role,
                "content": content,
                "sources": sources or [],
                "mode": mode,
                "created_at": datetime.utcnow().isoformat()
            }
            if session_id not in self._local_history:
                self._local_history[session_id] = []
            self._local_history[session_id].append(entry)
            return message_id

        await self.connect()

        message_id = str(uuid.uuid4())
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO conversation_history 
                (id, session_id, role, content, sources, mode)
                VALUES ($1, $2, $3, $4, $5, $6)
                """,
                uuid.UUID(message_id),
                uuid.UUID(session_id),
                role,
                content,
                json.dumps(sources or []),
                mode
            )

        return message_id
    
    async def get_conversation_history(
        self,
        session_id: str,
        limit: int = 10
    ) -> List[Dict[str, Any]]:
        """Get recent conversation history for a session"""
        # In-memory fallback
        if not self.database_url:
            rows = self._local_history.get(session_id, [])[-limit:]
            messages = []
            for row in rows:
                messages.append({
                    "role": row["role"],
                    "content": row["content"],
                    "sources": row.get("sources", []),
                    "mode": row.get("mode", "full_book"),
                    "timestamp": row.get("created_at")
                })
            return messages

        await self.connect()

        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT role, content, sources, mode, created_at
                FROM conversation_history
                WHERE session_id = $1
                ORDER BY created_at DESC
                LIMIT $2
                """,
                uuid.UUID(session_id),
                limit
            )

        # Return in chronological order
        messages = []
        for row in reversed(rows):
            messages.append({
                "role": row["role"],
                "content": row["content"],
                "sources": json.loads(row["sources"]) if row["sources"] else [],
                "mode": row["mode"],
                "timestamp": row["created_at"].isoformat()
            })

        return messages
    
    async def add_feedback(
        self,
        session_id: str,
        message_id: str,
        rating: int,
        feedback_text: str = None
    ):
        """Add user feedback for a response"""
        await self.connect()
        
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO chat_feedback 
                (session_id, message_id, rating, feedback_text)
                VALUES ($1, $2, $3, $4)
                """,
                uuid.UUID(session_id),
                uuid.UUID(message_id),
                rating,
                feedback_text
            )
    
    async def health_check(self) -> bool:
        """Check if database is accessible"""
        try:
            if not self.database_url:
                return False

            await self.connect()
            async with self.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
            return True
        except Exception:
            return False


# Singleton instance
_database_service = None


def get_database_service() -> DatabaseService:
    """Get singleton database service instance"""
    global _database_service
    if _database_service is None:
        _database_service = DatabaseService()
    return _database_service
