import React, { useState } from 'react';
import { useAuth } from './AuthContext';

const OnboardingWizard = ({ isOpen, onClose }) => {
  const { updateProfile } = useAuth();
  const [step, setStep] = useState(1);
  const [formData, setFormData] = useState({
    expertiseLevel: 'beginner',
    hasNvidiaGpu: false,
    osType: 'windows'
  });

  if (!isOpen) return null;

  const handleSubmit = () => {
    updateProfile(formData);
    onClose();
  };

  return (
    <div style={{
      position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
      backgroundColor: 'rgba(0,0,0,0.5)', display: 'flex', justifyContent: 'center', alignItems: 'center', zIndex: 1000
    }}>
      <div style={{ backgroundColor: 'white', padding: '2rem', borderRadius: '8px', maxWidth: '500px', width: '100%', color: 'black' }}>
        <h2>Personalize Your Learning</h2>
        
        {step === 1 && (
          <div>
            <h3>Step 1: Experience Level</h3>
            <p>How familiar are you with ROS 2 and Robotics?</p>
            <select 
              value={formData.expertiseLevel} 
              onChange={(e) => setFormData({...formData, expertiseLevel: e.target.value})}
              style={{ width: '100%', padding: '8px', marginBottom: '1rem' }}
            >
              <option value="beginner">Beginner (New to Robotics)</option>
              <option value="intermediate">Intermediate (Used ROS before)</option>
              <option value="advanced">Advanced (Building complex systems)</option>
            </select>
            <button onClick={() => setStep(2)} style={btnStyle}>Next</button>
          </div>
        )}

        {step === 2 && (
          <div>
            <h3>Step 2: Hardware Setup</h3>
            <p>Simulation requires specific hardware. What are you using?</p>
            
            <label style={{ display: 'block', marginBottom: '10px' }}>
              <input 
                type="checkbox" 
                checked={formData.hasNvidiaGpu}
                onChange={(e) => setFormData({...formData, hasNvidiaGpu: e.target.checked})}
              />
              I have an NVIDIA GPU (RTX 2060 or better)
            </label>

            <label style={{ display: 'block', marginBottom: '1rem' }}>
              Operating System:
              <select 
                value={formData.osType} 
                onChange={(e) => setFormData({...formData, osType: e.target.value})}
                style={{ marginLeft: '10px', padding: '4px' }}
              >
                <option value="windows">Windows</option>
                <option value="linux">Linux (Ubuntu)</option>
                <option value="mac">macOS</option>
              </select>
            </label>

            <button onClick={() => setStep(1)} style={{...btnStyle, backgroundColor: '#ccc', marginRight: '10px'}}>Back</button>
            <button onClick={handleSubmit} style={btnStyle}>Finish</button>
          </div>
        )}
      </div>
    </div>
  );
};

const btnStyle = {
  padding: '10px 20px',
  backgroundColor: '#2563eb',
  color: 'white',
  border: 'none',
  borderRadius: '4px',
  cursor: 'pointer'
};

export default OnboardingWizard;
