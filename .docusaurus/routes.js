import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'fac'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'be2'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '320'),
            routes: [
              {
                path: '/docs/AUTH_SYSTEM_DESIGN',
                component: ComponentCreator('/docs/AUTH_SYSTEM_DESIGN', 'd09'),
                exact: true
              },
              {
                path: '/docs/capstone-project/capstone-intro',
                component: ComponentCreator('/docs/capstone-project/capstone-intro', 'd2e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/demo-auth',
                component: ComponentCreator('/docs/demo-auth', '27f'),
                exact: true
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/rclpy-python-agents',
                component: ComponentCreator('/docs/module-1-ros2/rclpy-python-agents', '86c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/ros2-architecture',
                component: ComponentCreator('/docs/module-1-ros2/ros2-architecture', '1ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/ros2-communication',
                component: ComponentCreator('/docs/module-1-ros2/ros2-communication', '78c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/ros2-intro',
                component: ComponentCreator('/docs/module-1-ros2/ros2-intro', '8c9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/urdf-humanoids',
                component: ComponentCreator('/docs/module-1-ros2/urdf-humanoids', '5e0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/digital-twin-intro',
                component: ComponentCreator('/docs/module-2-digital-twin/digital-twin-intro', '424'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/gazebo-setup',
                component: ComponentCreator('/docs/module-2-digital-twin/gazebo-setup', 'cf2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/physics-fundamentals',
                component: ComponentCreator('/docs/module-2-digital-twin/physics-fundamentals', 'd9c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/sensor-simulation',
                component: ComponentCreator('/docs/module-2-digital-twin/sensor-simulation', 'b0c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/unity-hri',
                component: ComponentCreator('/docs/module-2-digital-twin/unity-hri', 'f94'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/urdf-sdf-humanoids',
                component: ComponentCreator('/docs/module-2-digital-twin/urdf-sdf-humanoids', 'a48'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-ecosystem',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-ecosystem', '720'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-intro',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-intro', '14c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-sim-photoreal',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-sim-photoreal', 'd9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/nav2-vslam',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/nav2-vslam', 'e52'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/synthetic-data-humanoids',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/synthetic-data-humanoids', '918'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/vla-intro',
                component: ComponentCreator('/docs/module-4-vla/vla-intro', '068'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/voice-cognitive-planning',
                component: ComponentCreator('/docs/module-4-vla/voice-cognitive-planning', '7c7'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
