import React, { createContext, useContext, useState, useEffect } from 'react';

// This is a placeholder context. 
// In production, this would connect to the Better-Auth client.

export const AuthContext = createContext({
  user: null,
  profile: null,
  isLoading: true,
  login: () => {},
  logout: () => {},
  updateProfile: () => {}
});

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [profile, setProfile] = useState(null);
  const [isLoading, setIsLoading] = useState(false); // Set to false for demo purposes

  // Mock Login Function
  const login = (email) => {
    setIsLoading(true);
    setTimeout(() => {
      setUser({ id: '123', email, name: 'Demo User' });
      // Default profile
      setProfile({
        role: 'student',
        expertiseLevel: 'beginner',
        hasNvidiaGpu: false,
        osType: 'windows'
      });
      setIsLoading(false);
    }, 500);
  };

  const logout = () => {
    setUser(null);
    setProfile(null);
  };

  const updateProfile = (newProfile) => {
    setProfile(prev => ({ ...prev, ...newProfile }));
  };

  return (
    <AuthContext.Provider value={{ user, profile, isLoading, login, logout, updateProfile }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => useContext(AuthContext);
