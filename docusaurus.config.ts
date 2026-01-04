import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// AI-Native Textbook: Physical AI & Humanoid Robotics

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'AI-Native Textbook for the Future of Human-Robot Symbiosis',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // Enable Mermaid diagram rendering
  markdown: {
    mermaid: true,
  },
  themes: [
    '@docusaurus/theme-mermaid',
    [
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
        docsRouteBasePath: '/docs',
        indexBlog: true,
        indexPages: true,
      },
    ],
  ],

  // GitHub Pages deployment configuration
  url: 'https://aibook-kohl.vercel.app/',
  baseUrl: '/',

  organizationName: 'panaversity',
  projectName: 'ai-native-book',
  trailingSlash: false,

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/panaversity/ai-native-book/tree/main/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
blog: {
  showReadingTime: true,
  feedOptions: {
    type: ['rss', 'atom'],
    xslt: true,
  },
  editUrl: 'https://github.com/panaversity/ai-native-book/tree/main/',
  blogTitle: 'Physical AI Insights',
  blogDescription: 'Latest developments in Physical AI and Humanoid Robotics',
},
sitemap: {
  changefreq: 'weekly',
  priority: 0.5,
  filename: 'sitemap.xml',
  ignorePatterns: [
    '/tags/**',
    '/search/**',
  ],
},
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  // PWA Plugin for offline support
  plugins: [
    [
      '@docusaurus/plugin-pwa',
      {
        debug: true,
        offlineModeActivationStrategies: [
          'appInstalled',
          'standalone',
          'queryString',
        ],
        pwaHead: [
          {
            tagName: 'link',
            rel: 'icon',
            href: '/img/icons/icon-192x192.png',
          },
          {
            tagName: 'link',
            rel: 'manifest',
            href: '/manifest.json',
          },
          {
            tagName: 'meta',
            name: 'theme-color',
            content: '#00d4ff',
          },
          {
            tagName: 'meta',
            name: 'apple-mobile-web-app-capable',
            content: 'yes',
          },
          {
            tagName: 'meta',
            name: 'apple-mobile-web-app-status-bar-style',
            content: '#0a0a0f',
          },
          {
            tagName: 'link',
            rel: 'apple-touch-icon',
            href: '/img/icons/icon-192x192.png',
          },
        ],
      },
    ],
  ],

  themeConfig: {
    image: 'img/social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      respectPrefersColorScheme: true,
    },
    announcementBar: {
      id: 'ai_native_learning',
      content: '🤖 <strong>AI-Native Learning:</strong> This textbook is designed for AI-assisted learning with Claude Code & Agents',
      backgroundColor: '#00d4ff',
      textColor: '#0a0a0a',
      isCloseable: true,
    },
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'curriculumSidebar',
          position: 'left',
          label: 'Curriculum',
        },
        { to: '/blog', label: 'Insights', position: 'left' },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/panaversity/ai-native-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Robotic Nervous System',
              to: '/docs/robotic-nervous-system',
            },
            {
              label: 'Digital Twin',
              to: '/docs/digital-twin',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Panaversity',
              href: 'https://panaversity.org',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/panaversity',
            },
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/panaversity/ai-native-book/discussions',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/panaversity/ai-native-book',
            },
            {
              label: 'Spec-Kit Plus',
              href: 'https://github.com/panaversity/spec-kit-plus',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus for AI-Native Learning.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'yaml', 'rust', 'cpp'],
    },
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
    mermaid: {
      theme: { light: 'neutral', dark: 'dark' },
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

