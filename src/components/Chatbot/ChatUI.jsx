import React, { useState } from 'react';

const ChatUI = () => {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");

  const backendUrl = "https://rag-chatbot-by-amna-production.up.railway.app/chat";
  const maxRetries = 3; // maximum retry attempts
  const retryDelay = 2000; // 2 seconds delay between retries

  const sendMessage = async (retryCount = 0) => {
    if (!input) return;
    const userMessage = { sender: "user", text: input };
    setMessages([...messages, userMessage]);
    setInput("");

    try {
      const res = await fetch(backendUrl, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ query: input })
      });

      // Handle API rate limit
      if (res.status === 429) {
        if (retryCount < maxRetries) {
          // Retry after a delay
          setTimeout(() => sendMessage(retryCount + 1), retryDelay);
          return;
        } else {
          setMessages((prev) => [
            ...prev,
            { sender: "bot", text: "⚠️ Server is busy or API key limit reached. Please wait a few seconds and try again." }
          ]);
          return;
        }
      }

      const data = await res.json();

      // Handle empty or invalid response
      if (!data.answer) {
        setMessages((prev) => [
          ...prev,
          { sender: "bot", text: "⚠️ Received empty response. Please try again." }
        ]);
        return;
      }

      setMessages((prev) => [...prev, { sender: "bot", text: data.answer }]);
    } catch (err) {
      console.error(err);
      setMessages((prev) => [
        ...prev,
        { sender: "bot", text: "⚠️ Could not connect to backend. Please check your internet or try again later." }
      ]);
    }
  };

  const styles = {
    container: {
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      zIndex: 9999,
      fontFamily: 'sans-serif',
    },
    toggle: {
      backgroundColor: '#4caf50',
      color: 'white',
      border: 'none',
      padding: '12px 18px',
      borderRadius: '50px',
      cursor: 'pointer',
      boxShadow: '0 4px 10px rgba(0,0,0,0.3)',
      transition: 'transform 0.2s ease, background-color 0.2s ease',
    },
    toggleHover: {
      transform: 'scale(1.1)',
      backgroundColor: '#45a049',
    },
    window: {
      marginTop: '10px',
      width: '320px',
      height: '420px',
      borderRadius: '15px',
      overflow: 'hidden',
      boxShadow: '0 6px 18px rgba(0,0,0,0.35)',
      display: 'flex',
      flexDirection: 'column',
      transform: open ? 'scale(1)' : 'scale(0.95)',
      opacity: open ? 1 : 0,
      transition: 'transform 0.3s ease, opacity 0.3s ease, box-shadow 0.3s ease',
      backgroundColor: '#fff',
    },
    messages: {
      flex: 1,
      padding: '12px',
      overflowY: 'auto',
      backgroundColor: '#f9f9f9',
      display: 'flex',
      flexDirection: 'column',
      gap: '6px',
      scrollBehavior: 'smooth',
    },
    inputContainer: {
      display: 'flex',
      borderTop: '1px solid #ddd',
      padding: '8px',
      backgroundColor: '#fff',
    },
    input: {
      flex: 1,
      padding: '10px 12px',
      border: '1px solid #ccc',
      borderRadius: '25px',
      outline: 'none',
      transition: 'border 0.2s ease, box-shadow 0.2s ease',
    },
    inputFocus: {
      border: '1px solid #4caf50',
      boxShadow: '0 0 5px rgba(76, 175, 80, 0.5)',
    },
    sendButton: {
      marginLeft: '8px',
      padding: '10px 16px',
      backgroundColor: '#4caf50',
      color: 'white',
      border: 'none',
      borderRadius: '25px',
      cursor: 'pointer',
      transition: 'transform 0.2s ease, background-color 0.2s ease',
    },
    sendButtonHover: {
      backgroundColor: '#45a049',
      transform: 'scale(1.05)',
    },
    msgUser: {
      alignSelf: 'flex-end',
      backgroundColor: '#e1ffc7',
      color: '#000',
      padding: '8px 12px',
      borderRadius: '15px',
      maxWidth: '80%',
      animation: 'fadeIn 0.3s ease',
    },
    msgBot: {
      alignSelf: 'flex-start',
      backgroundColor: '#d0f0fd',
      color: '#000',
      padding: '8px 12px',
      borderRadius: '15px',
      maxWidth: '80%',
      animation: 'fadeIn 0.3s ease',
    },
    '@keyframes fadeIn': {
      from: { opacity: 0, transform: 'translateY(10px)' },
      to: { opacity: 1, transform: 'translateY(0)' },
    },
  };
  

  return (
    <div style={styles.container}>
      <button style={styles.toggle} onClick={() => setOpen(!open)}>Chat</button>
      {open && (
        <div style={styles.window}>
          <div style={styles.messages}>
            {messages.map((m, i) => (
              <div key={i} style={m.sender === "user" ? styles.msgUser : styles.msgBot}>
                {m.text}
              </div>
            ))}
          </div>
          <div style={styles.inputContainer}>
            <input
              style={styles.input}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && sendMessage()}
              placeholder="Type a message..."
            />
            <button style={styles.sendButton} onClick={() => sendMessage()}>Send</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatUI;
