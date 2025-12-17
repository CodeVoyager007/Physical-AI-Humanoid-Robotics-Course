import React, { useRef } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl'; // Import useBaseUrl
import styles from './index.module.css';
import { useIsVisible } from '../theme/useIntersectionObserver';

function HomepageHeader() {
  const videoSrc = useBaseUrl('/Futuristic_Robot_Face_Video_Generated.mp4');
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroGlow}></div>
      <div className="container">
        <div className="row">
          <div className={`col col--7 ${styles.heroTextContainer}`}>
            <h1 className={styles.heroTitle}>
              Build Robots that <span className="gradient-text">Think</span>
            </h1>
            <p className={styles.heroSubtitle}>
              Master Physical AI with the world's most advanced, open-source robotics handbook. Dive into ROS 2, NVIDIA Isaac Sim, and Vision-Language Models.
            </p>
            <div className={styles.buttons}>
              <Link
                className={`button button--primary button--lg ${styles.glowingButton}`}
                to="/docs/category/foundations">
                Start Building →
              </Link>
            </div>
          </div>
          <div className={`col col--5 ${styles.heroImageContainer}`}>
            <video 
              src={videoSrc} 
              autoPlay 
              muted 
              loop 
              playsInline 
              className={styles.floatingElement} 
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function ModuleCard({title, description, link}) {
  const ref = useRef();
  const isVisible = useIsVisible(ref);

  return (
    <div ref={ref} className={`col col--6 margin-bottom--lg animate-on-scroll ${isVisible ? 'is-visible' : ''}`}>
      <div className="card-wrapper">
        <div className="card">
          <div className="card__header">
            <h3>{title}</h3>
          </div>
          <div className="card__body">
            <p>{description}</p>
          </div>
          <div className="card__footer">
            <Link
              className="button button--secondary button--block"
              to={link}>
              Explore Module
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}

function HomepageModules() {
  const modules = [
    {
      title: 'The Nervous System (ROS 2)',
      description: 'Learn the fundamentals of the Robot Operating System 2.',
      link: '/docs/category/ros-2'
    },
    {
      title: 'Digital Twins (Simulation)',
      description: 'Master robot simulation with URDF, Gazebo, and high-fidelity rendering.',
      link: '/docs/category/simulation'
    },
    {
      title: 'The AI Brain (NVIDIA Isaac)',
      description: 'Dive into NVIDIA\'s powerful platform for simulation and AI training.',
      link: '/docs/category/isaac'
    },
    {
      title: 'Vision-Language-Action (VLA)',
      description: 'Explore the cutting edge of AI that understands and acts on language.',
      link: '/docs/category/vla'
    },
  ];
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {modules.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Build Robots that Think"
      description="A comprehensive textbook teaching ROS 2, NVIDIA Isaac Sim, Gazebo, and Vision-Language-Action (VLA) Models.">
      <HomepageHeader />
      <main>
        <HomepageModules />
      </main>
    </Layout>
  );
}