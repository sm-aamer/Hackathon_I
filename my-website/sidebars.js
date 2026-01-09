// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Define our modular sidebar structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1/chapter-1-ros2-fundamentals',
        'module-1/chapter-2-python-agents',
        'module-1/chapter-3-urdf-essentials'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-2/chapter-1-gazebo-basics',
        'module-2/chapter-2-unity-interaction',
        'module-2/chapter-3-sensor-simulation'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-3/quickstart',
        'module-3/chapter-1-isaac-sim-essentials',
        'module-3/chapter-2-isaac-ros-integration',
        'module-3/chapter-3-nav2-humanoid-navigation'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-4/intro',
        'module-4/quickstart',
        'module-4/chapter-1-voice-to-action',
        'module-4/chapter-2-cognitive-planning',
        'module-4/chapter-3-autonomous-humanoid'
      ],
    },
    {
      type: 'category',
      label: 'Deployment & Optimization',
      collapsible: true,
      collapsed: true,
      items: [
        'deployment-optimization/intro',
        'deployment-optimization/getting-started-speed-insights'
      ],
    },
  ],
};

export default sidebars;