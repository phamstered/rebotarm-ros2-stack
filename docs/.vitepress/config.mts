import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: 'reBotArm B601 ROS 2 控制栈',
  description:
    'ROS 2 Humble arm control stack for reBotArm B601 — ros2_control, micro-ROS, Gazebo, MoveIt, CAN driver',
  base: '/rebotarm-ros2-stack/',
  cleanUrls: true,
  ignoreDeadLinks: true,
  themeConfig: {
    nav: [
      { text: '介绍', link: '/' },
      { text: '架构', link: '/architecture' },
      { text: '构建与运行', link: '/build-run' },
      { text: '控制栈', link: '/controllers' },
      { text: 'CI', link: '/ci' },
      {
        text: 'GitHub',
        link: 'https://github.com/phamstered/rebotarm-ros2-stack',
      },
    ],
    sidebar: [
      {
        text: '开始使用',
        items: [
          { text: '项目介绍', link: '/' },
          { text: '架构与包', link: '/architecture' },
          { text: '构建与运行', link: '/build-run' },
          { text: '控制栈与后端', link: '/controllers' },
        ],
      },
      {
        text: '参考',
        items: [
          { text: 'CI 说明', link: '/ci' },
          { text: 'ros2_control 关联机制', link: '/ros2_control_association' },
        ],
      },
    ],
    socialLinks: [
      {
        icon: 'github',
        link: 'https://github.com/phamstered/rebotarm-ros2-stack',
      },
    ],
    outline: { level: [2, 3] },
    search: { provider: 'local' },
    footer: {
      message: 'User docs for rebotarm-ros2-stack (ROS 2 control / arm software stack)',
      copyright: 'Grounded in src/ packages; historical teleop-only narrative is not authoritative',
    },
  },
})
