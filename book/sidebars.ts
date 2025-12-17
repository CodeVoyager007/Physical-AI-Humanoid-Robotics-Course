import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'The Awakening (Foundations)',
      link: {
        type: 'generated-index',
        title: 'Foundations',
        slug: '/category/foundations'
      },
      items: [
        'foundations/embodied-intelligence',
        'foundations/hardware-stack',
        'foundations/environment-setup',
        'foundations/linux-for-robotics',
      ],
    },
    {
      type: 'category',
      label: 'The Nervous System (ROS 2)',
      link: {
        type: 'generated-index',
        title: 'ROS 2 Fundamentals',
        slug: '/category/ros-2'
      },
      items: [
        'ros2/architecture',
        'ros2/communication',
        'ros2/services-actions',
        'ros2/python-agents',
        'ros2/launch-systems',
        'ros2/debugging',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twins (Simulation)',
      link: {
        type: 'generated-index',
        title: 'Simulation',
        slug: '/category/simulation'
      },
      items: [
        'sim/urdf-basics',
        'sim/modeling-bipeds',
        'sim/gazebo-physics',
        'sim/sensors',
        'sim/unity-bridge',
      ],
    },
    {
      type: 'category',
      label: 'The AI Brain (NVIDIA Isaac)',
      link: {
        type: 'generated-index',
        title: 'NVIDIA Isaac',
        slug: '/category/isaac'
      },
      items: [
        'isaac/omniverse-intro',
        'isaac/importing-robots',
        'isaac/synthetic-data',
        'isaac/vslam',
        'isaac/navigation2',
        'isaac/reinforcement-learning',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      link: {
        type: 'generated-index',
        title: 'VLA Models',
        slug: '/category/vla'
      },
      items: [
        'vla/generative-robotics',
        'vla/voice-command',
        'vla/cognitive-planning',
        'vla/vla-models',
        'vla/zero-shot',
      ],
    },
    {
      type: 'category',
      label: 'The Capstone & Deployment',
      link: {
        type: 'generated-index',
        title: 'Capstone',
        slug: '/category/capstone'
      },
      items: [
        'capstone/project-design',
        'capstone/sim-to-real',
        'capstone/safety-ethics',
        'capstone/deployment',
        'capstone/future',
      ],
    },
  ],
};

export default sidebars;
