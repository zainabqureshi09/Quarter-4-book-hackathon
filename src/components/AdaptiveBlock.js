import React from 'react';
import { useAuth } from './AuthContext';

/**
 * AdaptiveBlock component hides/shows content based on user profile.
 * 
 * Props:
 * - level: 'beginner' | 'intermediate' | 'advanced' (Content difficulty)
 * - requiresGpu: boolean (If true, requires profile.hasNvidiaGpu)
 * - children: React Node
 */
const AdaptiveBlock = ({ level, requiresGpu, children }) => {
  const { user, profile, isLoading } = useAuth();

  if (isLoading) return <div>Loading personalization...</div>;

  // If not logged in, show a teaser or default content
  if (!user) {
    return (
      <div style={{ border: '1px dashed #ccc', padding: '10px', margin: '10px 0', backgroundColor: '#f9f9f9' }}>
        <p><em>Please <a href="#" onClick={(e) => {e.preventDefault(); alert("Please implement the Login modal!");}}>sign in</a> to view this personalized content.</em></p>
        {/* Render children hidden to preserve anchors for build and TOC */}
        <div style={{ display: 'none' }}>{children}</div>
      </div>
    );
  }

  // GPU Check
  if (requiresGpu && !profile?.hasNvidiaGpu) {
    return (
      <div style={{ borderLeft: '4px solid #f59e0b', padding: '10px', backgroundColor: '#fffbeb', color: '#92400e' }}>
        <strong>Hardware Warning:</strong> This section requires an NVIDIA GPU. Your profile indicates you do not have one. 
        <br/>
        <em>You can proceed, but performance may be slow or simulation might fail.</em>
        <details>
            <summary>View Anyway</summary>
            {children}
        </details>
      </div>
    );
  }

  // Level Check
  // Logic: 
  // - If user is 'beginner', they see 'beginner' content normally.
  // - If user is 'beginner', 'intermediate'/'advanced' content is collapsed by default.
  // - If user is 'advanced', they see everything (or maybe hide beginner stuff? Let's keep it simple: Show all, but highlight relevant).
  
  const userLevel = profile?.expertiseLevel || 'beginner';
  const levels = { beginner: 1, intermediate: 2, advanced: 3 };
  
  const contentLevelVal = levels[level] || 1;
  const userLevelVal = levels[userLevel] || 1;

  // If content is harder than user level, collapse it
  if (contentLevelVal > userLevelVal) {
     return (
        <details style={{ margin: '1rem 0', padding: '0.5rem', border: '1px solid #ddd', borderRadius: '4px' }}>
            <summary><strong>{level.charAt(0).toUpperCase() + level.slice(1)} Content</strong> (Click to expand)</summary>
            <div style={{ marginTop: '10px' }}>
                {children}
            </div>
        </details>
     );
  }

  // Otherwise render normally
  return (
    <div className={`adaptive-block level-${level}`}>
      {children}
    </div>
  );
};

export default AdaptiveBlock;
