/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module-1-ros2/ros2-intro',
        'module-1-ros2/ros2-architecture',
        'module-1-ros2/ros2-communication',
        'module-1-ros2/rclpy-python-agents',
        'module-1-ros2/urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/digital-twin-intro',
        'module-2-digital-twin/physics-fundamentals',
        'module-2-digital-twin/gazebo-setup',
        'module-2-digital-twin/urdf-sdf-humanoids',
        'module-2-digital-twin/sensor-simulation',
        'module-2-digital-twin/unity-hri',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      items: [
        'module-3-ai-robot-brain/isaac-intro',
        'module-3-ai-robot-brain/isaac-ecosystem',
        'module-3-ai-robot-brain/isaac-sim-photoreal',
        'module-3-ai-robot-brain/synthetic-data-humanoids',
        'module-3-ai-robot-brain/nav2-vslam',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      items: [
        'module-4-vla/vla-intro',
        'module-4-vla/voice-cognitive-planning',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone-project/capstone-intro'],
    },
  ],
};

export default sidebars;
