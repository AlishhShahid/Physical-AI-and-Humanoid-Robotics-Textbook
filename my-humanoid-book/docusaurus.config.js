
// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Humanoid Robotics',
  tagline: 'Bridging the gap between AI and physical robots. From ROS2 to Isaac Sim and VLA.',
  favicon: 'img/logo.svg', // You can remove or replace with your own

  url: 'https://alishbashahid.github.io',
  baseUrl: '/',

  organizationName: 'AlishbaShahid',
  projectName: 'humanoid-robotics-book',

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/AlishbaShahid/humanoid-robotics-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: { type: ['rss', 'atom'], xslt: true },
          editUrl:
            'https://github.com/AlishbaShahid/humanoid-robotics-book/tree/main/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig: ({
    image: 'img/docusaurus-social-card.jpg',
   
      colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Robotics',
      logo: undefined, // Logo removed
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Docs',
        },
        {
          href: 'https://github.com/AlishbaShahid/humanoid-robotics-book',
          label: 'GitHub',
          position: 'right',
        },
        {
          to: '/about',
          label: 'About',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'light',
      links: [
        {
          title: 'Documentation',
          items: [
            { label: 'Intro', to: '/docs/intro' },
            { label: 'Modules', to: '/docs/category/module-1-the-ros2-nervous-system' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub', href: 'https://github.com/AlishbaShahid/humanoid-robotics-book' },
          ],
        },
        {
          title: 'Project',
          items: [
            { label: 'About', to: '/about' },
            { label: 'License', to: '/docs/intro' },
          ],
        },
      ],
      copyright: `Made with ❤️ by Alishba Shahid`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  }),
};

export default config;
