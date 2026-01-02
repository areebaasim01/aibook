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
      {/* Animated background orbs */}
      <div className={styles.heroOrbs}>
        <div className={styles.orb1}></div>
        <div className={styles.orb2}></div>
        <div className={styles.orb3}></div>
      </div>

      {/* Grid pattern overlay */}
      <div className={styles.gridPattern}></div>

      <div className="container">
        <div className={styles.heroContent}>
          <span className={styles.heroTag}>
            <span className={styles.heroTagDot}></span>
            🤖 AI-Native Learning Platform
          </span>
          <Heading as="h1" className={styles.heroTitle}>
            Physical AI &<br />
            <span className={styles.heroGradient}>Humanoid Robotics</span>
          </Heading>
          <p className={styles.heroSubtitle}>
            Master the future of human-robot symbiosis with AI-assisted learning.
            Build intelligent systems that <strong>sense</strong>, <strong>think</strong>, and <strong>act</strong> in the real world.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--lg', styles.buttonPrimary)}
              to="/docs/intro">
              <span>Start Learning</span>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M5 12h14M12 5l7 7-7 7" />
              </svg>
            </Link>
            <Link
              className={clsx('button button--lg', styles.buttonSecondary)}
              to="/docs/robotic-nervous-system">
              View Curriculum
            </Link>
          </div>
          <div className={styles.heroStats}>
            <div className={styles.stat}>
              <span className={styles.statNumber}>4</span>
              <span className={styles.statLabel}>Core Modules</span>
            </div>
            <div className={styles.statDivider}></div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>16</span>
              <span className={styles.statLabel}>Chapters</span>
            </div>
            <div className={styles.statDivider}></div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>50+</span>
              <span className={styles.statLabel}>Code Examples</span>
            </div>
            <div className={styles.statDivider}></div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>∞</span>
              <span className={styles.statLabel}>AI Assistance</span>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

type ModuleCardItem = {
  number: string;
  title: string;
  subtitle: string;
  icon: string;
  color: string;
  chapters: string[];
  link: string;
};

const ModuleList: ModuleCardItem[] = [
  {
    number: '01',
    title: 'Robotic Nervous System',
    subtitle: 'ROS 2 & Middleware',
    icon: '',
    color: '#00d4ff',
    chapters: ['ROS 2 Architecture', 'Python with rclpy', 'URDF Robot Models'],
    link: '/docs/robotic-nervous-system',
  },
  {
    number: '02',
    title: 'Digital Twin',
    subtitle: 'Simulation & Physics',
    icon: '',
    color: '#7c3aed',
    chapters: ['Gazebo Physics', 'Unity Rendering', 'Sensor Simulation'],
    link: '/docs/digital-twin',
  },
  {
    number: '03',
    title: 'AI-Robot Brain',
    subtitle: 'Perception & Navigation',
    icon: '',
    color: '#10b981',
    chapters: ['Isaac Sim', 'Visual SLAM', 'Nav2 Navigation'],
    link: '/docs/ai-robot-brain',
  },
  {
    number: '04',
    title: 'Vision-Language-Action',
    subtitle: 'Multimodal Intelligence',
    icon: '',
    color: '#f59e0b',
    chapters: ['Voice Pipeline', 'LLM Planning', 'Autonomous Humanoid'],
    link: '/docs/vision-language-action',
  },
];

function ModuleCard({ number, title, subtitle, icon, color, chapters, link }: ModuleCardItem) {
  return (
    <Link to={link} className={styles.moduleCard} style={{ '--module-color': color } as React.CSSProperties}>
      <div className={styles.moduleHeader}>
        <span className={styles.moduleNumber}>{number}</span>
        <span className={styles.moduleIcon}>{icon}</span>
      </div>
      <h3 className={styles.moduleTitle}>{title}</h3>
      <p className={styles.moduleSubtitle}>{subtitle}</p>
      <ul className={styles.moduleChapters}>
        {chapters.map((chapter, idx) => (
          <li key={idx}>{chapter}</li>
        ))}
      </ul>
      <div className={styles.moduleArrow}>
        <span>Start Module</span>
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M5 12h14M12 5l7 7-7 7" />
        </svg>
      </div>
    </Link>
  );
}

function ModulesSection() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>Curriculum</span>
          <Heading as="h2">Four Pillars of Physical AI</Heading>
          <p>A comprehensive journey from middleware fundamentals to autonomous humanoid systems</p>
        </div>
        <div className={styles.modulesGrid}>
          {ModuleList.map((module, idx) => (
            <ModuleCard key={idx} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Human-Robot Symbiosis',
    icon: '',
    description: 'Build systems where humans, AI agents, and robots work together seamlessly. Focus on partnership, not replacement.',
  },
  {
    title: 'AI-Native Pedagogy',
    icon: '',
    description: 'Content designed for AI-assisted learning. Every chapter is structured for interactive, hands-on exploration with Claude or GPT.',
  },
  {
    title: 'Production-Ready Code',
    icon: '',
    description: 'Theory balanced with deployable code. Build real perception, planning, and control systems that work on actual hardware.',
  },
  {
    title: 'Future-Proof Skills',
    icon: '',
    description: 'Curriculum aligned with the Future of Work. Master skills in robotics and AI that will define the next decade.',
  },
];

function Feature({ title, icon, description }: FeatureItem) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>💡 Philosophy</span>
          <Heading as="h2">Why This Textbook?</Heading>
          <p>Built for the next generation of roboticists and AI engineers</p>
        </div>
        <div className={styles.featuresGrid}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

const testimonials = [
  {
    quote: "The integration of ROS 2 with modern AI frameworks is exactly what the robotics field needed. This textbook bridges that gap beautifully.",
    author: "Dr. Sarah Chen",
    role: "Robotics Researcher, Stanford",
    avatar: "",
  },
  {
    quote: "Finally, a resource that teaches physical AI from first principles while keeping up with the latest in LLMs and vision models.",
    author: "Marcus Williams",
    role: "Senior Engineer, Boston Dynamics",
    avatar: "",
  },
  {
    quote: "The capstone project alone is worth the entire course. Building an autonomous humanoid end-to-end was transformative.",
    author: "Yuki Tanaka",
    role: "ML Engineer, NVIDIA",
    avatar: "",
  },
];

function TestimonialsSection() {
  return (
    <section className={styles.testimonials}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>💬 Testimonials</span>
          <Heading as="h2">What Learners Say</Heading>
          <p>Join thousands of engineers mastering Physical AI</p>
        </div>
        <div className={styles.testimonialsGrid}>
          {testimonials.map((testimonial, idx) => (
            <div key={idx} className={styles.testimonialCard}>
              <div className={styles.testimonialQuote}>"{testimonial.quote}"</div>
              <div className={styles.testimonialAuthor}>
                <span className={styles.testimonialAvatar}>{testimonial.avatar}</span>
                <div>
                  <div className={styles.testimonialName}>{testimonial.author}</div>
                  <div className={styles.testimonialRole}>{testimonial.role}</div>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function TechStackSection() {
  const technologies = [
    { name: 'ROS 2', icon: '🤖' },
    { name: 'Gazebo', icon: '🌍' },
    { name: 'Isaac Sim', icon: '🎮' },
    { name: 'Python', icon: '🐍' },
    { name: 'OpenAI', icon: '🧠' },
    { name: 'Nav2', icon: '🗺️' },
    { name: 'Unity', icon: '🎯' },
    { name: 'URDF', icon: '📐' },
  ];

  return (
    <section className={styles.techStack}>
      <div className="container">
        <p className={styles.techStackLabel}>Technologies you'll master</p>
        <div className={styles.techStackGrid}>
          {technologies.map((tech, idx) => (
            <div key={idx} className={styles.techItem}>
              <span>{tech.icon}</span>
              <span>{tech.name}</span>
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
          <span className={styles.ctaEmoji}></span>
          <Heading as="h2">Ready to Build the Future?</Heading>
          <p>
            Start your journey into Physical AI today. Learn to build robots that
            see, understand, and act — powered by the latest in AI and robotics.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button button--lg', styles.buttonPrimary)}
              to="/docs/intro">
              Begin Your Journey
            </Link>
            <a
              className={clsx('button button--lg', styles.buttonGhost)}
              href="https://github.com/yourusername/ai-native-book"
              target="_blank"
              rel="noopener noreferrer">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z" />
              </svg>
              Star on GitHub
            </a>
          </div>
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
        <TechStackSection />
        <ModulesSection />
        <FeaturesSection />
        <TestimonialsSection />
        <CTASection />
      </main>
    </Layout>
  );
}
