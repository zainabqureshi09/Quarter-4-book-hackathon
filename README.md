# Physical AI & Humanoid Robotics Textbook 🤖📚

**An AI-Native, Spec-Driven, and Interactive Textbook Platform.**

[![Tech Stack](https://skillicons.dev/icons?i=react,ts,python,fastapi,postgres,docker,vercel)](https://docusaurus.io/)

This repository hosts the "Physical AI & Humanoid Robotics" textbook. It is a **Fullstack Application** combining a Docusaurus static site with a dynamic Python backend for Authentication, Personalization, and AI Chat.

---

## 📂 1. Repository Structure

```bash
physical-ai-textbook/
├── docs/                      # 📚 Content Modules (Markdown/MDX)
│   ├── module-1-ros2/         # Module 1: ROS 2
│   ├── module-2-digital-twin/ # Module 2: Simulation
│   ├── module-3-ai-robot-brain/ # Module 3: Isaac Sim
│   ├── module-4-vla/          # Module 4: VLA
│   └── capstone-project/      # Capstone Guide
├── src/                       # ⚛️ React Components
│   ├── components/            # Custom UI (UrduToggle, Auth, Chat)
│   └── theme/                 # Docusaurus Layouts
├── backend/                   # 🧠 FastAPI Service (Create this folder if missing)
│   ├── app/
│   │   ├── main.py            # API Entrypoint
│   │   ├── auth.py            # Better-Auth Integration
│   │   └── rag.py             # Qdrant RAG Chatbot
│   └── requirements.txt       # Python Dependencies
├── book.spec.yaml             # 📋 Spec-Kit Plus Definition
├── docusaurus.config.js       # ⚙️ Site Configuration
└── package.json               # 📦 Frontend Dependencies
```

---

## 🛠️ 2. Prerequisites

Before you begin, ensure you have:

*   **Software**:
    *   [Node.js](https://nodejs.org/) (v18 or higher)
    *   [Python](https://www.python.org/) (v3.10 or higher)
    *   [Git](https://git-scm.com/)
*   **Cloud Accounts** (Free Tiers):
    *   **Database**: [Neon](https://neon.tech) (Postgres)
    *   **Vector Store**: [Qdrant](https://qdrant.tech) (Cloud)
    *   **Hosting**: [Vercel](https://vercel.com) (Frontend) & [Render](https://render.com) (Backend)
    *   **AI**: [OpenAI API Key](https://platform.openai.com)

---

## 🚀 3. Quick Start (Local Development)

Follow these commands to get the project running on your machine.

### Step 3.1: Clone and Setup Git
```bash
# Initialize git if you haven't already
git init
git add .
git commit -m "Initial commit"

# Or clone if this is an existing repo
# git clone https://github.com/username/physical-ai-textbook.git
# cd physical-ai-textbook
```

### Step 3.2: Frontend Setup (Docusaurus)
```bash
# Install Node dependencies
npm install

# Start the local development server
npm start
```
*The site will open at `http://localhost:3000`.*

### Step 3.3: Backend Setup (FastAPI)
*If the `backend/` folder does not exist, create it.*

```bash
# 1. Create backend directory
mkdir backend
cd backend

# 2. Create virtual environment
python -m venv venv

# 3. Activate virtual environment
# Windows:
venv\Scripts\activate
# Mac/Linux:
# source venv/bin/activate

# 4. Install dependencies
pip install fastapi uvicorn better-auth-python qdrant-client openai psycopg2-binary python-dotenv

# 5. Run the server (Requires app/main.py to exist)
uvicorn app.main:app --reload --port 8000
```

---

## ☁️ 4. Deployment Guide

We will deploy the **Backend** first, then the **Frontend**.

### Phase 1: Cloud Services Setup

1.  **Neon (Postgres)**:
    *   Create a new project.
    *   Copy the **Connection String** (e.g., `postgres://user:pass@...`).
2.  **Qdrant (Vector DB)**:
    *   Create a cluster.
    *   Get the **URL** and **API Key**.
3.  **OpenAI**:
    *   Generate a new **API Key**.

### Phase 2: Backend Deployment (Render)

We use **Render** because it supports Python natively.

1.  Push your code to **GitHub**.
2.  Go to [Render Dashboard](https://dashboard.render.com/) -> **New** -> **Web Service**.
3.  Connect your GitHub repository.
4.  **Settings**:
    *   **Root Directory**: `backend`
    *   **Build Command**: `pip install -r requirements.txt`
    *   **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port 10000`
5.  **Environment Variables** (Add these in Render):
    *   `DATABASE_URL`: *(Your Neon Connection String)*
    *   `QDRANT_URL`: *(Your Qdrant URL)*
    *   `QDRANT_API_KEY`: *(Your Qdrant Key)*
    *   `OPENAI_API_KEY`: *(Your OpenAI Key)*
    *   `BETTER_AUTH_SECRET`: *(Generate a random string)*
    *   `FRONTEND_URL`: `https://your-project.vercel.app` (Add this after Phase 3)

### Phase 3: Frontend Deployment (Vercel)

**Recommended**: Vercel is optimized for Next.js/React/Docusaurus.

1.  Go to [Vercel Dashboard](https://vercel.com/dashboard) -> **Add New** -> **Project**.
2.  Import your GitHub repository.
3.  **Framework Preset**: Select **Docusaurus**.
4.  **Build Settings** (Default is usually correct):
    *   Command: `npm run build`
    *   Output Directory: `build`
5.  **Environment Variables**:
    *   `REACT_APP_API_URL`: `https://your-backend-on-render.com` (The URL from Phase 2)
6.  Click **Deploy**.

#### Alternative: GitHub Pages (Static Only)
*Note: This option is easier but may require extra configuration for client-side routing and won't support server-side features natively without the external backend.*

1.  Open `docusaurus.config.js` and set:
    *   `url`: `'https://username.github.io'`
    *   `baseUrl`: `'/repo-name/'`
2.  Run the deploy command:
    ```bash
    GIT_USER=your-username npm run deploy
    ```

---

## 🔌 5. Backend Endpoints (API Reference)

When your backend is running, it should expose these endpoints:

| Method | Endpoint | Description |
| :--- | :--- | :--- |
| `GET` | `/` | Health check |
| `POST` | `/api/auth/*` | Better-Auth routes (signin, signup, session) |
| `POST` | `/api/chat` | RAG Chatbot query endpoint |
| `GET` | `/api/user/profile` | Get user expertise level & hardware stats |
| `POST` | `/api/user/profile` | Update user expertise level |

---

## 🧪 6. Verification Checklist

- [ ] **Site Loads**: Homepage is visible at your Vercel URL.
- [ ] **Modules Visible**: Sidebar shows Modules 1–4.
- [ ] **Urdu Toggle**: Clicking "Urdu" shows translated text.
- [ ] **Authentication**: You can Sign Up / Sign In (requires Backend + Neon).
- [ ] **Chatbot**: Sending a message returns a response (requires Backend + Qdrant).

---

## 📝 7. Contributing Content

1.  **Edit Content**: Modify files in `docs/`.
2.  **Add Images**: Place images in `static/img/` and reference via `![](/img/image.png)`.
3.  **Update Spec**: If adding a new chapter, update `book.spec.yaml` first.

---

*Created by Zainab Ayaz for the Physical AI & Humanoid Robotics Hackathon.*
