import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

// This wrapper adds the ChatWidget to every page
export default function Root({ children }) {
    return (
        <>
            {children}
            <ChatWidget
                apiUrl={process.env.NODE_ENV === 'production'
                    ? 'https://your-api-domain.com'
                    : 'http://localhost:8000'
                }
                position="bottom-right"
            />
        </>
    );
}
