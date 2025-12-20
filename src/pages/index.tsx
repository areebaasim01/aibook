import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HeroSection() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <span className={styles.heroTag}>🤖 AI-Native Learning</span>
          <Heading as="h1" className="hero__title">
            Physical AI &<br />Humanoid Robotics
          </Heading>
          <p className="hero__subtitle">
            Master the future of human-robot symbiosis with AI-assisted learning.
            Build intelligent systems that sense, think, and act in the real world.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning →
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/part-1-foundations/introduction-to-physical-ai">
              View Curriculum
            </Link>
          </div>
          <div className={styles.heroStats}>
            <div className={styles.stat}>
              <span className={styles.statNumber}>13</span>
              <span className={styles.statLabel}>Chapters</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>40+</span>
              <span className={styles.statLabel}>Code Examples</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>4</span>
              <span className={styles.statLabel}>Learning Paths</span>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

type FeatureItem = {
  title: string;
  emoji: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Human-Agent-Robot Symbiosis',
    emoji: '🤝',
    description: (
      <>
        Learn to build systems where humans, AI agents, and robots work together
        seamlessly. Focus on partnership, not replacement.
      </>
    ),
  },
  {
    title: 'AI-Native Pedagogy',
    emoji: '🧠',
    description: (
      <>
        Content designed for AI-assisted learning with Claude Code. Every chapter
        is structured for interactive, hands-on exploration.
      </>
    ),
  },
  {
    title: 'Practical Rigor',
    emoji: '⚙️',
    description: (
      <>
        Theory balanced with deployable code. Build real perception, planning,
        and control systems that work on actual hardware.
      </>
    ),
  },
  {
    title: 'Future-Ready Skills',
    emoji: '🚀',
    description: (
      <>
        Curriculum aligned with the Future of Work. Master skills in robotics
        and AI that will define the next decade of technology.
      </>
    ),
  },
];

function Feature({ title, emoji, description }: FeatureItem) {
  return (
    <div className={clsx('col col--3', styles.feature)}>
      <div className={styles.featureCard}>
        <div className={styles.featureEmoji}>{emoji}</div>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2">Core Principles</Heading>
          <p>Built for the next generation of roboticists and AI engineers</p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function CurriculumSection() {
  const parts = [
    {
      title: 'Part 1: Foundations',
      emoji: '🏗️',
      chapters: ['Introduction to Physical AI', 'Humanoid Robotics Fundamentals', 'AI Agent Architecture'],
      color: '#00d4ff',
    },
    {
      title: 'Part 2: Core Technologies',
      emoji: '⚙️',
      chapters: ['Sensor Integration', 'Motor Control', 'Computer Vision', 'Natural Language'],
      color: '#7c3aed',
    },
    {
      title: 'Part 3: Intelligence Layer',
      emoji: '🧠',
      chapters: ['Decision Making', 'Learning from Demonstration', 'Multi-Agent Coordination'],
      color: '#10b981',
    },
    {
      title: 'Part 4: Integration',
      emoji: '🚀',
      chapters: ['System Integration', 'Safety & Ethics', 'Deployment & Operations'],
      color: '#f59e0b',
    },
  ];

  return (
    <section className={styles.curriculum}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2">Curriculum Overview</Heading>
          <p>A comprehensive journey from fundamentals to deployment</p>
        </div>
        <div className={styles.curriculumGrid}>
          {parts.map((part, idx) => (
            <div key={idx} className={styles.curriculumCard} style={{ '--accent-color': part.color } as React.CSSProperties}>
              <span className={styles.curriculumEmoji}>{part.emoji}</span>
              <h3>{part.title}</h3>
              <ul>
                {part.chapters.map((chapter, cidx) => (
                  <li key={cidx}>{chapter}</li>
                ))}
              </ul>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2">Ready to Build the Future?</Heading>
          <p>
            Join thousands of learners mastering Physical AI and Humanoid Robotics.
            Start your journey today with AI-assisted learning.
          </p>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Begin Your Journey →
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="AI-Native Textbook"
      description="Master Physical AI and Humanoid Robotics with AI-assisted learning. Build intelligent systems that sense, think, and act.">
      <HeroSection />
      <main>
        <FeaturesSection />
        <CurriculumSection />
        <CTASection />
      </main>
    </Layout>
  );
}
