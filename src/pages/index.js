import React, {useEffect, useState} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

/* --------------------------------------------------
   COMPONENT: Animated Background Particles 
   -------------------------------------------------- */
function ParticleBackground() {
  // Simple CSS-based particle effect for performance
  // In a real production app, we might use react-particles-js
  return (
    <div className={styles.particleContainer}>
      {[...Array(20)].map((_, i) => (
        <div 
          key={i} 
          className={styles.particle}
          style={{
            left: `${Math.random() * 100}%`,
            top: `${Math.random() * 100}%`,
            animationDelay: `${Math.random() * 5}s`,
            animationDuration: `${10 + Math.random() * 10}s`
          }}
        />
      ))}
    </div>
  );
}

/* --------------------------------------------------
   COMPONENT: Terminal Typewriter 
   -------------------------------------------------- */
function Terminal() {
  const [text, setText] = useState('');
  const fullText = "ros2 launch humanoid_bringup brain.launch.py --mode=autonomous";

  useEffect(() => {
    let index = 0;
    const interval = setInterval(() => {
      setText(fullText.slice(0, index));
      index++;
      if (index > fullText.length) clearInterval(interval);
    }, 50);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="terminal-window">
      <div className="terminal-header">
        <div className="terminal-dot red"></div>
        <div className="terminal-dot yellow"></div>
        <div className="terminal-dot green"></div>
      </div>
      <div className="terminal-body">
        <div className="command-line">
          <span className="prompt">user@robot:~$</span>
          <span>{text}<span className="cursor"></span></span>
        </div>
        {text.length === fullText.length && (
          <div style={{marginTop: '10px', color: '#4ade80'}}>
            [INFO] Starting Isaac ROS VSLAM...<br/>
            [INFO] Loading VLA Model (GPT-4o)...<br/>
            [SUCCESS] Humanoid Online. Ready for commands.
          </div>
        )}
      </div>
    </div>
  );
}

/* --------------------------------------------------
   COMPONENT: Hero Section 
   -------------------------------------------------- */
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <ParticleBackground />
      <div className="container" style={{position: 'relative', zIndex: 10}}>
        <div className={styles.heroContent}>
          <div className="row">
            <div className="col col--12 text--center">
              <h1 className="hero__title">
                BUILD THE <span className="highlight">ROBOT BRAIN</span>
              </h1>
              <p className="hero__subtitle">
                The First AI-Native Textbook for <br/>
                <strong>Physical AI & Humanoid Robotics</strong>
              </p>
              
              <div className={styles.buttons}>
                <Link
                  className="button button--secondary button--lg"
                  to="/docs/intro">
                  Start Learning ⚡
                </Link>
                <Link
                  className="button button--outline button--lg"
                  style={{marginLeft: '1rem'}}
                  to="/docs/capstone-project/capstone-intro">
                  View Capstone 🤖
                </Link>
              </div>

              <Terminal />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

/* --------------------------------------------------
   COMPONENT: Features 
   -------------------------------------------------- */
function Feature({title, description, icon, delay}) {
  return (
    <div className={clsx('col col--4')} style={{animationDelay: delay}}>
      <div className="featureCard">
        <div className="text--center">
          <span className="featureIcon">{icon}</span>
          <Heading as="h3" style={{marginBottom: '10px'}}>{title}</Heading>
        </div>
        <div className="text--center">
          <p style={{color: 'var(--color-text-dim)'}}>{description}</p>
        </div>
      </div>
    </div>
  );
}

/* --------------------------------------------------
   COMPONENT: Interactive Roadmap 
   -------------------------------------------------- */
function Roadmap() {
  return (
    <div className={styles.section} style={{background: 'var(--color-surface)', padding: '5rem 0'}}>
      <div className="container">
        <Heading as="h2" className="text--center" style={{fontSize: '2.5rem', marginBottom: '3rem'}}>
          <span style={{borderBottom: '3px solid var(--color-primary)'}}>Curriculum Roadmap</span>
        </Heading>
        <div className="row">
          <div className="col col--3">
            <div className={styles.stepCard}>
              <div className={styles.stepNumber}>01</div>
              <h3>ROS 2 Core</h3>
              <p>Master the middleware. Nodes, Topics, Services, and Actions in Python.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.stepCard}>
              <div className={styles.stepNumber}>02</div>
              <h3>Digital Twin</h3>
              <p>Simulate physics in Isaac Sim. URDFs, Sensors, and Real-to-Sim.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.stepCard}>
              <div className={styles.stepNumber}>03</div>
              <h3>AI Brain</h3>
              <p>Visual SLAM, Nav2, and Deep Learning for Perception.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.stepCard}>
              <div className={styles.stepNumber}>04</div>
              <h3>VLA Agents</h3>
              <p>Voice-to-Action with LLMs. The frontier of Embodied AI.</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home | ${siteConfig.title}`}
      description="The ultimate guide to building autonomous humanoid robots.">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <Feature 
                title="AI-Native Architecture"
                icon="🧠"
                description="Built for the Transformer era. Learn how to integrate GPT-4, Whisper, and CLIP directly into your robot's control loop."
                delay="0s"
              />
              <Feature 
                title="Photoreal Simulation"
                icon="🎮"
                description="Stop debugging hardware. Train agents in NVIDIA Isaac Sim with ray-tracing and physics-accurate sensor data."
                delay="0.2s"
              />
              <Feature 
                title="Industry Standards"
                icon="⚡"
                description="Zero toy code. We use production-grade ROS 2 Humble, Docker, and CI/CD pipelines used by Tesla and Boston Dynamics."
                delay="0.4s"
              />
            </div>
          </div>
        </section>
        <Roadmap />
      </main>
    </Layout>
  );
}
