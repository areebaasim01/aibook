import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './ChatWidget.module.css';

interface SourceChunk {
    content: string;
    source: string;
    page_title: string;
    relevance_score: number;
    chunk_id: string;
}

interface ChatMessage {
    role: 'user' | 'assistant';
    content: string;
    sources?: SourceChunk[];
    timestamp: Date;
}

interface ChatWidgetProps {
    apiUrl?: string;
    position?: 'bottom-right' | 'bottom-left';
}

export default function ChatWidget({
    apiUrl = 'http://localhost:8000',
    position = 'bottom-right'
}: ChatWidgetProps) {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<ChatMessage[]>([]);
    const [inputValue, setInputValue] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [sessionId, setSessionId] = useState<string | null>(null);
    const [selectedText, setSelectedText] = useState<string>('');
    const [selectedMode, setSelectedMode] = useState(false);
    const [showSources, setShowSources] = useState<number | null>(null);

    const messagesEndRef = useRef<HTMLDivElement>(null);
    const inputRef = useRef<HTMLInputElement>(null);

    // Scroll to bottom when messages change
    useEffect(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages]);

    // Handle text selection on the page
    useEffect(() => {
        const handleSelection = () => {
            const selection = window.getSelection();
            if (selection && selection.toString().trim().length > 10) {
                setSelectedText(selection.toString().trim());
            }
        };

        document.addEventListener('mouseup', handleSelection);
        return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const sendMessage = useCallback(async () => {
        if (!inputValue.trim() || isLoading) return;

        const userMessage: ChatMessage = {
            role: 'user',
            content: inputValue,
            timestamp: new Date(),
        };

        setMessages(prev => [...prev, userMessage]);
        setInputValue('');
        setIsLoading(true);

        try {
            const endpoint = selectedMode && selectedText
                ? `${apiUrl}/chat/selected`
                : `${apiUrl}/chat`;

            const response = await fetch(endpoint, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: inputValue,
                    session_id: sessionId,
                    mode: selectedMode ? 'selected_text' : 'full_book',
                    selected_text: selectedMode ? selectedText : undefined,
                    selected_source: selectedMode ? window.location.pathname : undefined,
                }),
            });

            if (!response.ok) {
                throw new Error('Failed to get response');
            }

            const data = await response.json();

            setSessionId(data.session_id);

            const assistantMessage: ChatMessage = {
                role: 'assistant',
                content: data.answer,
                sources: data.sources,
                timestamp: new Date(),
            };

            setMessages(prev => [...prev, assistantMessage]);
        } catch (error) {
            console.error('Chat error:', error);
            const errorMessage: ChatMessage = {
                role: 'assistant',
                content: 'Sorry, I encountered an error. Please try again.',
                timestamp: new Date(),
            };
            setMessages(prev => [...prev, errorMessage]);
        } finally {
            setIsLoading(false);
        }
    }, [inputValue, isLoading, selectedMode, selectedText, sessionId, apiUrl]);

    const handleKeyPress = (e: React.KeyboardEvent) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    };

    const clearChat = () => {
        setMessages([]);
        setSessionId(null);
    };

    const toggleSelectedMode = () => {
        setSelectedMode(!selectedMode);
    };

    return (
        <>
            {/* Chat Toggle Button */}
            <button
                className={`${styles.toggleButton} ${styles[position]}`}
                onClick={() => setIsOpen(!isOpen)}
                aria-label={isOpen ? 'Close chat' : 'Open chat'}
            >
                {isOpen ? (
                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                        <path d="M18 6L6 18M6 6l12 12" />
                    </svg>
                ) : (
                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
                    </svg>
                )}
            </button>

            {/* Chat Window */}
            {isOpen && (
                <div className={`${styles.chatWindow} ${styles[position]}`}>
                    {/* Header */}
                    <div className={styles.header}>
                        <div className={styles.headerTitle}>
                            <span className={styles.headerIcon}>🤖</span>
                            <div>
                                <h3>Physical AI Assistant</h3>
                                <span className={styles.headerSubtitle}>Ask about the textbook</span>
                            </div>
                        </div>
                        <button onClick={clearChat} className={styles.clearButton} title="Clear chat">
                            🗑️
                        </button>
                    </div>

                    {/* Selected Text Mode Toggle */}
                    {selectedText && (
                        <div className={styles.selectedTextBanner}>
                            <label className={styles.modeToggle}>
                                <input
                                    type="checkbox"
                                    checked={selectedMode}
                                    onChange={toggleSelectedMode}
                                />
                                <span>Ask about selected text</span>
                            </label>
                            {selectedMode && (
                                <div className={styles.selectedPreview}>
                                    "{selectedText.slice(0, 100)}..."
                                </div>
                            )}
                        </div>
                    )}

                    {/* Messages */}
                    <div className={styles.messages}>
                        {messages.length === 0 && (
                            <div className={styles.welcome}>
                                <h4>👋 Welcome!</h4>
                                <p>Ask me anything about Physical AI & Humanoid Robotics.</p>
                                <div className={styles.suggestions}>
                                    <button onClick={() => setInputValue("What is ROS 2?")}>
                                        What is ROS 2?
                                    </button>
                                    <button onClick={() => setInputValue("Explain URDF")}>
                                        Explain URDF
                                    </button>
                                    <button onClick={() => setInputValue("How does Nav2 work?")}>
                                        How does Nav2 work?
                                    </button>
                                </div>
                            </div>
                        )}

                        {messages.map((message, index) => (
                            <div
                                key={index}
                                className={`${styles.message} ${styles[message.role]}`}
                            >
                                <div className={styles.messageContent}>
                                    {message.content}
                                </div>

                                {/* Sources */}
                                {message.sources && message.sources.length > 0 && (
                                    <div className={styles.sourcesContainer}>
                                        <button
                                            className={styles.sourcesToggle}
                                            onClick={() => setShowSources(showSources === index ? null : index)}
                                        >
                                            📚 {message.sources.length} source{message.sources.length > 1 ? 's' : ''}
                                        </button>

                                        {showSources === index && (
                                            <div className={styles.sourcesList}>
                                                {message.sources.map((source, sIdx) => (
                                                    <a
                                                        key={sIdx}
                                                        href={source.source}
                                                        className={styles.sourceLink}
                                                    >
                                                        <span className={styles.sourceTitle}>{source.page_title}</span>
                                                        <span className={styles.sourceScore}>
                                                            {Math.round(source.relevance_score * 100)}% relevant
                                                        </span>
                                                    </a>
                                                ))}
                                            </div>
                                        )}
                                    </div>
                                )}
                            </div>
                        ))}

                        {isLoading && (
                            <div className={`${styles.message} ${styles.assistant}`}>
                                <div className={styles.typing}>
                                    <span></span>
                                    <span></span>
                                    <span></span>
                                </div>
                            </div>
                        )}

                        <div ref={messagesEndRef} />
                    </div>

                    {/* Input */}
                    <div className={styles.inputContainer}>
                        <input
                            ref={inputRef}
                            type="text"
                            value={inputValue}
                            onChange={(e) => setInputValue(e.target.value)}
                            onKeyPress={handleKeyPress}
                            placeholder={selectedMode ? "Ask about selected text..." : "Ask about the book..."}
                            className={styles.input}
                            disabled={isLoading}
                        />
                        <button
                            onClick={sendMessage}
                            disabled={!inputValue.trim() || isLoading}
                            className={styles.sendButton}
                        >
                            ➤
                        </button>
                    </div>
                </div>
            )}
        </>
    );
}
