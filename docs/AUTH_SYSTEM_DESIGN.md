# Authentication & Personalization System Design
**Technology Stack**: Docusaurus (Frontend), Better-Auth (Auth Engine), Neon Serverless Postgres (Database), Node.js/Hono (Backend API).

## 1. Executive Summary

This specification outlines the architecture for adding a robust Signup/Signin system to the "Physical AI & Humanoid Robotics" textbook platform. The goal is to move from a static content viewer to an intelligent, personalized learning platform that adapts content based on the user's hardware (e.g., "Do you have an NVIDIA GPU?") and expertise level.

We will use **Better-Auth** for its modern, secure, and extensible authentication capabilities, backed by **Neon Postgres** for serverless scalability.

---

## 2. System Architecture

Since Docusaurus is a static-site generator (SSG) / Single Page Application (SPA) hybrid, it cannot host the server-side logic required for secure authentication and database interactions.

We propose a **Headless Auth Architecture**:

1.  **Frontend (The Textbook)**: Docusaurus (React) running on the client.
2.  **Backend (The Brain)**: A lightweight Node.js API Service (using Hono or Express) hosted on Vercel/Render/Railway.
3.  **Database**: Neon Serverless Postgres.

```mermaid
graph TD
    User[User Browser]
    Frontend[Docusaurus App]
    AuthAPI[Auth Server (Node.js + Better-Auth)]
    DB[(Neon Postgres)]
    Agent[Personalization Agent]

    User --> Frontend
    Frontend -- "1. Login/Signup (Client Client)" --> AuthAPI
    AuthAPI -- "2. Verify/Store" --> DB
    Frontend -- "3. Get Profile" --> AuthAPI
    AuthAPI -- "4. Fetch Data" --> DB
    Frontend -- "5. Render Adaptive Content" --> User
    Agent -- "6. Analyze Progress" --> DB
```

---

## 3. Database Schema (Neon Postgres)

We will use a relational schema. Better-Auth requires core tables (`user`, `session`, `account`, `verification`). We will extend this with a `profile` table.

### 3.1 Core Auth Tables (Better-Auth Standard)

```sql
CREATE TABLE "user" (
    "id" TEXT NOT NULL PRIMARY KEY,
    "name" TEXT NOT NULL,
    "email" TEXT NOT NULL UNIQUE,
    "emailVerified" BOOLEAN NOT NULL,
    "image" TEXT,
    "createdAt" TIMESTAMP NOT NULL,
    "updatedAt" TIMESTAMP NOT NULL
);

CREATE TABLE "session" (
    "id" TEXT NOT NULL PRIMARY KEY,
    "expiresAt" TIMESTAMP NOT NULL,
    "ipAddress" TEXT,
    "userAgent" TEXT,
    "userId" TEXT NOT NULL REFERENCES "user"("id")
);

CREATE TABLE "account" (
    "id" TEXT NOT NULL PRIMARY KEY,
    "accountId" TEXT NOT NULL,
    "providerId" TEXT NOT NULL,
    "userId" TEXT NOT NULL REFERENCES "user"("id"),
    "accessToken" TEXT,
    "refreshToken" TEXT,
    "idToken" TEXT,
    "expiresAt" TIMESTAMP,
    "password" TEXT -- If using credential auth
);

CREATE TABLE "verification" (
    "id" TEXT NOT NULL PRIMARY KEY,
    "identifier" TEXT NOT NULL,
    "value" TEXT NOT NULL,
    "expiresAt" TIMESTAMP NOT NULL
);
```

### 3.2 User Profile & Hardware Context

This table stores the "Context" needed for the Personalization Agent.

```sql
CREATE TABLE "user_profile" (
    "userId" TEXT NOT NULL PRIMARY KEY REFERENCES "user"("id"),
    
    -- Expertise Level
    "role" TEXT NOT NULL DEFAULT 'student', -- student, instructor, researcher
    "expertiseLevel" TEXT NOT NULL DEFAULT 'beginner', -- beginner, intermediate, advanced
    
    -- Hardware Context (Crucial for Robotics)
    "hasNvidiaGpu" BOOLEAN DEFAULT FALSE,
    "gpuModel" TEXT, -- e.g., "RTX 3060", "Orin Nano"
    "osType" TEXT, -- "windows", "linux", "mac"
    "hasRealRobot" BOOLEAN DEFAULT FALSE,
    "robotType" TEXT, -- e.g., "Unitree Go2", "Franka Emika"
    
    -- Learning Goals
    "interests" TEXT[], -- ["ros2", "sim2real", "vla"]
    
    "createdAt" TIMESTAMP DEFAULT NOW(),
    "updatedAt" TIMESTAMP DEFAULT NOW()
);
```

---

## 4. API Design (Backend)

The backend service will expose the Better-Auth routes and custom profile endpoints.

### 4.1 Auth Routes (Automated by Better-Auth)
- `POST /api/auth/sign-up`
- `POST /api/auth/sign-in`
- `POST /api/auth/sign-out`
- `GET /api/auth/session`

### 4.2 Profile Routes (Custom)

**POST /api/onboarding**
*Triggered after initial signup to collect hardware details.*
```json
{
  "expertiseLevel": "intermediate",
  "hasNvidiaGpu": true,
  "gpuModel": "RTX 4090",
  "osType": "linux"
}
```

**GET /api/agent/recommendations**
*Used by the frontend to fetch personalization hints.*
*Response:*
```json
{
  "showAdvancedPhysics": true,
  "suggestedModule": "module-3-ai-robot-brain",
  "hardwareWarning": null
}
```

---

## 5. Frontend Integration (Docusaurus)

### 5.1 Installation
```bash
npm install better-auth @better-auth/react axios
```

### 5.2 Auth Client Setup
Create `src/lib/auth-client.ts`:
```typescript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
    baseURL: "https://your-auth-api.com" // Point to your backend
});
```

### 5.3 Onboarding Wizard Component
Create `src/components/OnboardingWizard.js`. This is a modal that appears if `user.profile` is missing.

```javascript
import React, { useState } from 'react';
import { authClient } from '../lib/auth-client';

export const OnboardingWizard = ({ onComplete }) => {
  const [formData, setFormData] = useState({
    level: 'beginner',
    gpu: false,
    os: 'windows'
  });

  const handleSubmit = async () => {
    // Call API to save profile
    await fetch('https://your-api.com/api/onboarding', {
      method: 'POST',
      body: JSON.stringify(formData)
    });
    onComplete();
  };

  return (
    <div className="modal">
      <h2>Welcome! Let's customize your textbook.</h2>
      
      <label>What is your ROS 2 experience?</label>
      <select onChange={e => setFormData({...formData, level: e.target.value})}>
        <option value="beginner">Beginner (What is a Node?)</option>
        <option value="intermediate">Intermediate (I use rclpy)</option>
        <option value="advanced">Advanced (I write middleware)</option>
      </select>

      <label>
        <input type="checkbox" onChange={e => setFormData({...formData, gpu: e.target.checked})} />
        Do you have an NVIDIA GPU?
      </label>

      <button onClick={handleSubmit}>Start Learning</button>
    </div>
  );
};
```

### 5.4 Personalization Context
Wrap the app in `src/theme/Root.js` with an `AuthProvider` that fetches the session and profile.

### 5.5 Adaptive Content Component
A component to show/hide content based on the profile.

```javascript
// src/components/AdaptiveBlock.js
import { useAuth } from '../hooks/useAuth'; // Custom hook

export const AdaptiveBlock = ({ level, requiresGpu, children }) => {
  const { user, profile } = useAuth();

  if (requiresGpu && !profile.hasNvidiaGpu) {
    return <div className="warning">Note: This section requires an NVIDIA GPU. You can skip or run in Cloud.</div>;
  }

  // If user is beginner, hide advanced blocks unless they toggle "Show All"
  if (level === 'advanced' && profile.expertiseLevel === 'beginner') {
    return <details><summary>Advanced Topic (Click to expand)</summary>{children}</details>;
  }

  return <div>{children}</div>;
};
```

---

## 6. Security Considerations

1.  **HttpOnly Cookies**: Better-Auth uses HttpOnly cookies by default for session management. This prevents XSS attacks from stealing tokens.
2.  **CORS**: The backend API must be configured to allow CORS *only* from the Docusaurus domain.
3.  **Role-Based Access Control (RBAC)**: The API should verify `user.role` before allowing administrative actions (like editing content, if implemented).
4.  **Data Minimization**: Only store hardware specs relevant to the curriculum. Do not collect PII beyond email/name.

## 7. Integration with Personalization Agent

The "Personalization Agent" is effectively an AI service that reads the `user_profile` and `user_progress` (future) to recommend the next best module.

**Integration Pattern:**
1.  **Direct DB Access**: The AI Agent (running as a cron job or microservice) has read-only access to the `user_profile` table.
2.  **Webhook**: When a user completes a quiz (in the textbook), a webhook `POST /api/progress` is sent. The Agent analyzes this and updates a `recommended_path` field in the DB.
3.  **UI Feedback**: The Docusaurus frontend queries `recommended_path` and highlights the next chapter in the sidebar.
