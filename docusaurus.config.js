// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  // 1. Site Metadata
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital Brains and Physical Bodies',
  favicon: 'img/favicon1.jpeg',

  // 2. Deployment Config (GitHub Pages ke liye)
  url: 'https://AamnaAnsari.github.io', // Jab deploy karein tab apna asli URL dalein
  baseUrl: '/physical-ai-book-by-amna/', 
  organizationName: 'AamnaAnsari', // Yahan apna GitHub username likhna
  projectName: 'physical-ai-book-by-amna', // Aapki repo ka naam
  trailingSlash: false,

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Agar "Edit this page" button hatana hai toh neeche wali line delete kardo
          editUrl: 'https://github.com/your-github-username/physical-ai-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics Book', // Top Left Corner Name
        logo: {
          alt: 'Robotics Logo',
          src: 'img/favicon1.jpeg', // Baad mein aap apna robot logo laga sakti hain
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Start Reading 📖', // Button ka naam change kar diya
          },
          // Blog link hata diya hai taake clean lage, wapas lana ho toh uncomment karein:
          // {to: '/blog', label: 'Updates', position: 'left'},
          {
            href: 'https://github.com/AamnaAnsari',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          // {
          //   title: 'Course Modules',
          //   items: [
          //     {
          //       label: 'Introduction',
          //       to: '/docs/Robotic',
          //     },
          //     {
          //       label: 'ROS 2 Basics',
          //       to: '/docs/book-chapters/02-ros2-basics',
          //     },
          //   ],
          // },
          {
            title: 'Tools & Tech',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://developer.nvidia.com/isaac-sim',
              },
              {
                label: 'OpenAI Whisper',
                href: 'https://github.com/openai/whisper',
              },
            ],
          },
          {
            title: 'Connect with me',
            items: [
              {
                label: 'GitHub Repository',
                href: 'https://github.com/AamnaAnsari',
              },

               {
                label: 'Linkedin',
                href: 'https://www.linkedin.com/in/aamna-ansari-57660a254/',
              },

               {
                label: 'X',
                href: 'https://x.com/AamnaAnsari4706',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook, Built with ❤️ by Aamna Ansari.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;