import React, { useState, useRef, useEffect } from 'react';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!inputValue.trim()) return;

    const userMessage = {
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch('https://saimaamjad-reg-chatbot.hf.space/v1/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userMessage.content
        })
      });

      if (!response.ok) {
        throw new Error(`Server error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      const assistantMessage = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        responseTime: data.response_time_ms,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Error fetching response:', err);
      setError(err.message || 'Failed to get response. Please try again.');

      // Add error message to chat
      const errorMessage = {
        role: 'error',
        content: `Error: ${err.message || 'Failed to connect to server. Make sure the backend is running on http://127.0.0.1:8000'}`,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearChat = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <>
      {/* Floating Button */}
      {!isOpen && (
        <button
          onClick={() => setIsOpen(true)}
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#2563eb',
            color: 'white',
            border: 'none',
            cursor: 'pointer',
            fontSize: '24px',
            boxShadow: '0 4px 12px rgba(37, 99, 235, 0.3)',
            zIndex: 1000,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            transition: 'transform 0.2s',
          }}
          onMouseEnter={(e) => e.target.style.transform = 'scale(1.1)'}
          onMouseLeave={(e) => e.target.style.transform = 'scale(1)'}
          title="Open Chat"
        >
          💬
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '380px',
            height: '550px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 8px 24px rgba(0, 0, 0, 0.2)',
            zIndex: 1000,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
          }}
        >
          {/* Header */}
          <div
            style={{
              backgroundColor: '#2563eb',
              color: 'white',
              padding: '16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
            }}
          >
            <h3 style={{ margin: 0, fontSize: '18px', fontWeight: '600' }}>
              Ask a Question
            </h3>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={handleClearChat}
                style={{
                  background: 'rgba(255, 255, 255, 0.2)',
                  border: 'none',
                  color: 'white',
                  cursor: 'pointer',
                  borderRadius: '4px',
                  padding: '4px 8px',
                  fontSize: '12px',
                }}
                title="Clear chat"
              >
                Clear
              </button>
              <button
                onClick={() => setIsOpen(false)}
                style={{
                  background: 'none',
                  border: 'none',
                  color: 'white',
                  fontSize: '24px',
                  cursor: 'pointer',
                  padding: 0,
                  lineHeight: 1,
                }}
                title="Close chat"
              >
                ×
              </button>
            </div>
          </div>

          {/* Messages Container */}
          <div
            style={{
              flex: 1,
              overflowY: 'auto',
              padding: '16px',
              backgroundColor: '#f8f9fa',
            }}
          >
            {messages.length === 0 && (
              <div style={{ textAlign: 'center', color: '#6c757d', marginTop: '20px' }}>
                <p>👋 Welcome! Ask me anything about the book.</p>
              </div>
            )}

            {messages.map((message, index) => (
              <div
                key={index}
                style={{
                  marginBottom: '12px',
                  display: 'flex',
                  justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start',
                }}
              >
                <div
                  style={{
                    maxWidth: '80%',
                    padding: '10px 14px',
                    borderRadius: '12px',
                    backgroundColor: message.role === 'user'
                      ? '#2563eb'
                      : message.role === 'error'
                      ? '#dc3545'
                      : 'white',
                    color: message.role === 'user' || message.role === 'error' ? 'white' : '#333',
                    boxShadow: '0 1px 2px rgba(0, 0, 0, 0.1)',
                  }}
                >
                  <div style={{ fontSize: '14px', lineHeight: '1.5' }}>
                    {message.content}
                  </div>
                  {message.citations && message.citations.length > 0 && (
                    <div style={{
                      marginTop: '8px',
                      fontSize: '12px',
                      color: '#6c757d',
                      borderTop: '1px solid #e9ecef',
                      paddingTop: '8px'
                    }}>
                      <strong>Sources:</strong>
                      {message.citations.map((citation, idx) => (
                        <div key={idx} style={{ marginTop: '4px' }}>
                          • {citation.chapter} {citation.section && `- ${citation.section}`}
                        </div>
                      ))}
                    </div>
                  )}
                  {message.responseTime && (
                    <div style={{
                      marginTop: '4px',
                      fontSize: '11px',
                      color: '#6c757d',
                      fontStyle: 'italic'
                    }}>
                      {message.responseTime}ms
                    </div>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div
                style={{
                  display: 'flex',
                  justifyContent: 'flex-start',
                  marginBottom: '12px',
                }}
              >
                <div
                  style={{
                    padding: '10px 14px',
                    borderRadius: '12px',
                    backgroundColor: 'white',
                    boxShadow: '0 1px 2px rgba(0, 0, 0, 0.1)',
                  }}
                >
                  <div style={{ fontSize: '14px', color: '#6c757d' }}>
                    Thinking...
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form
            onSubmit={handleSubmit}
            style={{
              padding: '16px',
              borderTop: '1px solid #e9ecef',
              backgroundColor: 'white',
            }}
          >
            <div style={{ display: 'flex', gap: '8px' }}>
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Type your question..."
                disabled={isLoading}
                style={{
                  flex: 1,
                  padding: '10px 12px',
                  border: '1px solid #dee2e6',
                  borderRadius: '8px',
                  fontSize: '14px',
                  outline: 'none',
                }}
                onFocus={(e) => e.target.style.borderColor = '#2563eb'}
                onBlur={(e) => e.target.style.borderColor = '#dee2e6'}
              />
              <button
                type="submit"
                disabled={isLoading || !inputValue.trim()}
                style={{
                  padding: '10px 20px',
                  backgroundColor: isLoading || !inputValue.trim() ? '#6c757d' : '#2563eb',
                  color: 'white',
                  border: 'none',
                  borderRadius: '8px',
                  cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer',
                  fontSize: '14px',
                  fontWeight: '600',
                }}
              >
                {isLoading ? '...' : 'Send'}
              </button>
            </div>
          </form>
        </div>
      )}
    </>
  );
};

export default Chatbot;
