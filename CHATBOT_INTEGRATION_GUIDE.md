# Chatbot Integration Guide for Docusaurus

This guide shows you how to integrate the floating chatbot UI into your existing Docusaurus site **without overwriting any content**.

## ✅ What's Already Done

1. **Chatbot Component Created**: `book-physical-ai/src/components/Chatbot.js`
2. **Backend CORS Configured**: Backend already allows `http://localhost:3000`

## 📋 Backend Setup

### Backend is at: `physical-humanoid-robotics-book/backend/`

The backend is already configured correctly:
- **CORS**: Allows `http://localhost:3000` (Docusaurus dev server)
- **Endpoint**: `http://127.0.0.1:8000/v1/query`
- **Method**: POST
- **Request Format**: `{ "query": "your question here" }`

### Start the Backend

```bash
# Navigate to backend folder
cd physical-humanoid-robotics-book/backend

# Activate virtual environment
# On Windows:
.venv\Scripts\activate

# On Mac/Linux:
source .venv/bin/activate

# Install dependencies (if not already installed)
pip install -r requirements.txt

# Start the FastAPI server
python -m src.main
```

The backend should start on `http://127.0.0.1:8000`

**Verify it's running**: Open `http://127.0.0.1:8000/health` in your browser.

## 📋 Frontend Setup

### Frontend is at: `book-physical-ai/`

### 1. Start Docusaurus Dev Server

```bash
# Navigate to frontend folder
cd book-physical-ai

# Install dependencies (if not already installed)
npm install

# Start dev server
npm start
```

Docusaurus should start on `http://localhost:3000`

### 2. Add Chatbot to Your Pages

You have **two options** to add the chatbot:

#### **Option A: Add to Specific Page (Recommended)**

Add the chatbot to any `.mdx` or `.js` page **without replacing content**:

**Example: `book-physical-ai/src/pages/index.js`**

```javascript
import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Chatbot from '@site/src/components/Chatbot';  // ADD THIS LINE
import styles from './index.module.css';

export default function Home() {
  return (
    <Layout
      title="Welcome"
      description="Physical AI and Humanoid Robotics">
      <main>
        {/* YOUR EXISTING CONTENT STAYS HERE */}
        <h1>Welcome to My Book</h1>
        <p>This is my existing content...</p>

        {/* ADD CHATBOT AT THE END */}
        <Chatbot />
      </main>
    </Layout>
  );
}
```

**For .mdx pages** (like `docs/intro.mdx`):

```mdx
---
sidebar_position: 1
---

# Introduction

Your existing content here...

More content...

{/* Add chatbot at the end */}
import Chatbot from '@site/src/components/Chatbot';

<Chatbot />
```

#### **Option B: Add Globally (All Pages)**

To add the chatbot to **every page**, create a wrapper component:

**Create: `book-physical-ai/src/theme/Root.js`**

```javascript
import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
```

This makes the chatbot available on all pages without modifying individual files.

## 🧪 Test the Integration

1. **Start Backend**: `cd physical-humanoid-robotics-book/backend && python -m src.main`
2. **Start Frontend**: `cd book-physical-ai && npm start`
3. **Open Browser**: Go to `http://localhost:3000`
4. **Look for floating button**: Bottom-right corner (💬)
5. **Click to open chat**
6. **Ask a question**: Type something and click "Send"

### Expected Behavior

- **Floating button** appears at bottom-right
- **Click button** → Chat window opens
- **Type question** → Click "Send"
- **Loading state**: Shows "Thinking..."
- **Response appears**: With answer, citations, and response time
- **Clear button**: Clears chat history
- **Close button** (×): Closes chat window

### Troubleshooting

#### ❌ "Failed to connect to server"

**Solution**: Make sure backend is running on `http://127.0.0.1:8000`

```bash
# Check if backend is running
curl http://127.0.0.1:8000/health
```

#### ❌ CORS Error in Browser Console

**Solution**: Verify CORS is configured in backend `.env`:

```bash
# physical-humanoid-robotics-book/backend/.env
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
```

Restart the backend after changing `.env`.

#### ❌ Component Not Found

**Solution**: Make sure you're using the correct import path:

```javascript
import Chatbot from '@site/src/components/Chatbot';
```

Docusaurus uses `@site` as an alias for the root directory.

## 📁 File Structure

```
chatbot/
├── book-physical-ai/                    # Docusaurus frontend
│   ├── src/
│   │   ├── components/
│   │   │   └── Chatbot.js              # ✅ Chatbot component (NEW)
│   │   ├── pages/
│   │   │   └── index.js                # Add <Chatbot /> here
│   │   └── theme/
│   │       └── Root.js                 # Optional: Global chatbot
│   └── package.json
│
└── physical-humanoid-robotics-book/    # Project root
    └── backend/
        ├── src/
        │   ├── main.py                 # FastAPI app (CORS configured)
        │   └── config.py               # CORS settings
        ├── .env                        # CORS_ORIGINS configured
        └── requirements.txt
```

## 🎨 Customization

The chatbot component is in `book-physical-ai/src/components/Chatbot.js`.

You can customize:

### Change Colors

```javascript
// Line 95: Floating button color
backgroundColor: '#007bff',  // Change to your brand color

// Line 164: Header color
backgroundColor: '#007bff',  // Change to match button
```

### Change Position

```javascript
// Line 89-90: Floating button position
bottom: '20px',  // Distance from bottom
right: '20px',   // Distance from right
```

### Change Size

```javascript
// Line 111: Chat window size
width: '380px',   // Chat width
height: '550px',  // Chat height
```

## 🚀 Production Deployment

Before deploying to production:

1. **Update CORS origins** in backend `.env`:
   ```bash
   CORS_ORIGINS=https://yourdomain.com,http://localhost:3000
   ```

2. **Update API URL** in `Chatbot.js`:
   ```javascript
   const response = await fetch('https://your-backend-url.com/v1/query', {
   ```

3. **Build Docusaurus**:
   ```bash
   cd book-physical-ai
   npm run build
   ```

## 📝 Summary

- ✅ Chatbot component created at `book-physical-ai/src/components/Chatbot.js`
- ✅ Backend CORS already configured for `http://localhost:3000`
- ✅ No existing content will be overwritten
- ✅ Add `<Chatbot />` to any page or globally via `Root.js`
- ✅ Floating button UI that doesn't interfere with content
- ✅ Full error handling and loading states included

## 🎯 Next Steps

1. Start backend: `cd physical-humanoid-robotics-book/backend && python -m src.main`
2. Start frontend: `cd book-physical-ai && npm start`
3. Add `<Chatbot />` to a page or create `Root.js` for global access
4. Test the chatbot and verify it works

Need help? Check the troubleshooting section above!
