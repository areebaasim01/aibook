import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

/**
 * AI-Native Textbook: Physical AI & Humanoid Robotics
 * Curriculum Sidebar Configuration
 */
const sidebars: SidebarsConfig = {
  curriculumSidebar: [
    'intro',
    {
      type: 'category',
      label: '🏗️ Part 1: Foundations',
      link: {
        type: 'generated-index',
        title: 'Foundations of Physical AI',
        description: 'Build your understanding of the core concepts in Physical AI and Humanoid Robotics.',
        slug: '/part-1-foundations',
      },
      items: [
        'part-1-foundations/introduction-to-physical-ai',
        'part-1-foundations/humanoid-robotics-fundamentals',
        'part-1-foundations/ai-agent-architecture',
      ],
    },
    {
      type: 'category',
      label: '⚙️ Part 2: Core Technologies',
      link: {
        type: 'generated-index',
        title: 'Core Technologies',
        description: 'Master the essential technologies that power humanoid robots.',
        slug: '/part-2-core-tech',
      },
      items: [
        'part-2-core-tech/sensor-integration',
        'part-2-core-tech/motor-control',
        'part-2-core-tech/computer-vision',
        'part-2-core-tech/natural-language-interfaces',
      ],
    },
    {
      type: 'category',
      label: '🧠 Part 3: Intelligence Layer',
      link: {
        type: 'generated-index',
        title: 'Intelligence Layer',
        description: 'Explore the AI systems that enable intelligent robot behavior.',
        slug: '/part-3-intelligence',
      },
      items: [
        'part-3-intelligence/decision-making',
        'part-3-intelligence/learning-from-demonstration',
        'part-3-intelligence/multi-agent-coordination',
      ],
    },
    {
      type: 'category',
      label: '🚀 Part 4: Integration & Deployment',
      link: {
        type: 'generated-index',
        title: 'Integration & Deployment',
        description: 'Learn to deploy and operate physical AI systems in the real world.',
        slug: '/part-4-integration',
      },
      items: [
        'part-4-integration/system-integration',
        'part-4-integration/safety-and-ethics',
        'part-4-integration/deployment-operations',
      ],
    },
  ],
};

export default sidebars;
