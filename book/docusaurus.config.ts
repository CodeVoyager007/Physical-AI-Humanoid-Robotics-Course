import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Sentient Machines',
  tagline: 'Engineering Physical AI',
  favicon: 'img/logo.png',

  // Set the production url of your site here
  url: 'https://CodeVoyager007.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/BOOK-WITH-RAGCHATBOT/',

  // GitHub pages deployment config.
  organizationName: 'CodeVoyager007',
  projectName: 'BOOK-WITH-RAGCHATBOT',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',
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
          editUrl:
            'https://github.com/CodeVoyager007/BOOK-WITH-RAGCHATBOT/tree/main/book/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig:
    {
      image: 'img/logo.png',
      navbar: {
        title: 'Sentient Machines',
        logo: {
          alt: 'Sentient Machines Logo',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Handbook',
          },
          {
            href: 'https://github.com/CodeVoyager007/BOOK-WITH-RAGCHATBOT',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [],
        copyright: `Copyright © ${new Date().getFullYear()} Sentient Machines. Built with Docusaurus.`,
      },
    } satisfies Preset.ThemeConfig,
};

export default config;
