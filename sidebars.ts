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
      label: '🔌 Module 1: Robotic Nervous System',
      link: {
        type: 'doc',
        id: 'robotic-nervous-system/index',
      },
      items: [
        'robotic-nervous-system/ros2-fundamentals',
        'robotic-nervous-system/python-bridging',
        'robotic-nervous-system/humanoid-urdf',
      ],
    },
    {
      type: 'category',
      label: '🪞 Module 2: Digital Twin',
      link: {
        type: 'doc',
        id: 'digital-twin/index',
      },
      items: [
        'digital-twin/physics-engines',
        'digital-twin/unity-rendering',
        'digital-twin/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: '🧠 Module 3: AI-Robot Brain',
      link: {
        type: 'doc',
        id: 'ai-robot-brain/index',
      },
      items: [
        'ai-robot-brain/isaac-sim',
        'ai-robot-brain/visual-slam',
        'ai-robot-brain/nav2-navigation',
      ],
    },
    {
      type: 'category',
      label: '🎯 Module 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'vision-language-action/index',
      },
      items: [
        'vision-language-action/voice-pipeline',
        'vision-language-action/cognitive-logic',
        'vision-language-action/capstone',
      ],
    },
  ],
};

export default sidebars;
