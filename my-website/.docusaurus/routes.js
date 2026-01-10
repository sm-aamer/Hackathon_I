import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/blog/',
    component: ComponentCreator('/blog/', '189'),
    exact: true
  },
  {
    path: '/blog/archive/',
    component: ComponentCreator('/blog/archive/', '1d9'),
    exact: true
  },
  {
    path: '/blog/authors/',
    component: ComponentCreator('/blog/authors/', '347'),
    exact: true
  },
  {
    path: '/blog/authors/all-sebastien-lorber-articles/',
    component: ComponentCreator('/blog/authors/all-sebastien-lorber-articles/', 'a25'),
    exact: true
  },
  {
    path: '/blog/authors/yangshun/',
    component: ComponentCreator('/blog/authors/yangshun/', 'c91'),
    exact: true
  },
  {
    path: '/blog/first-blog-post/',
    component: ComponentCreator('/blog/first-blog-post/', '08c'),
    exact: true
  },
  {
    path: '/blog/long-blog-post/',
    component: ComponentCreator('/blog/long-blog-post/', '447'),
    exact: true
  },
  {
    path: '/blog/mdx-blog-post/',
    component: ComponentCreator('/blog/mdx-blog-post/', 'bcc'),
    exact: true
  },
  {
    path: '/blog/tags/',
    component: ComponentCreator('/blog/tags/', 'e17'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus/',
    component: ComponentCreator('/blog/tags/docusaurus/', '350'),
    exact: true
  },
  {
    path: '/blog/tags/facebook/',
    component: ComponentCreator('/blog/tags/facebook/', '187'),
    exact: true
  },
  {
    path: '/blog/tags/hello/',
    component: ComponentCreator('/blog/tags/hello/', '046'),
    exact: true
  },
  {
    path: '/blog/tags/hola/',
    component: ComponentCreator('/blog/tags/hola/', 'bf6'),
    exact: true
  },
  {
    path: '/blog/welcome/',
    component: ComponentCreator('/blog/welcome/', 'a11'),
    exact: true
  },
  {
    path: '/markdown-page/',
    component: ComponentCreator('/markdown-page/', '54d'),
    exact: true
  },
  {
    path: '/docs/',
    component: ComponentCreator('/docs/', 'e52'),
    routes: [
      {
        path: '/docs/',
        component: ComponentCreator('/docs/', '569'),
        routes: [
          {
            path: '/docs/tags/',
            component: ComponentCreator('/docs/tags/', 'dc9'),
            exact: true
          },
          {
            path: '/docs/tags/autonomous-robotics/',
            component: ComponentCreator('/docs/tags/autonomous-robotics/', '870'),
            exact: true
          },
          {
            path: '/docs/tags/bipedal-locomotion/',
            component: ComponentCreator('/docs/tags/bipedal-locomotion/', '1e4'),
            exact: true
          },
          {
            path: '/docs/tags/cognitive-planning/',
            component: ComponentCreator('/docs/tags/cognitive-planning/', '968'),
            exact: true
          },
          {
            path: '/docs/tags/command-interpreter/',
            component: ComponentCreator('/docs/tags/command-interpreter/', '25b'),
            exact: true
          },
          {
            path: '/docs/tags/command-processing/',
            component: ComponentCreator('/docs/tags/command-processing/', 'cc1'),
            exact: true
          },
          {
            path: '/docs/tags/full-pipeline/',
            component: ComponentCreator('/docs/tags/full-pipeline/', '807'),
            exact: true
          },
          {
            path: '/docs/tags/gait-aware-navigation/',
            component: ComponentCreator('/docs/tags/gait-aware-navigation/', '2b4'),
            exact: true
          },
          {
            path: '/docs/tags/gpt/',
            component: ComponentCreator('/docs/tags/gpt/', '65e'),
            exact: true
          },
          {
            path: '/docs/tags/gpu-acceleration/',
            component: ComponentCreator('/docs/tags/gpu-acceleration/', '618'),
            exact: true
          },
          {
            path: '/docs/tags/humanoid-navigation/',
            component: ComponentCreator('/docs/tags/humanoid-navigation/', '4bb'),
            exact: true
          },
          {
            path: '/docs/tags/humanoid/',
            component: ComponentCreator('/docs/tags/humanoid/', 'e05'),
            exact: true
          },
          {
            path: '/docs/tags/isaac-ros/',
            component: ComponentCreator('/docs/tags/isaac-ros/', 'df8'),
            exact: true
          },
          {
            path: '/docs/tags/isaac-sim/',
            component: ComponentCreator('/docs/tags/isaac-sim/', 'eee'),
            exact: true
          },
          {
            path: '/docs/tags/llm/',
            component: ComponentCreator('/docs/tags/llm/', '4f6'),
            exact: true
          },
          {
            path: '/docs/tags/manipulation/',
            component: ComponentCreator('/docs/tags/manipulation/', '944'),
            exact: true
          },
          {
            path: '/docs/tags/multi-step-planning/',
            component: ComponentCreator('/docs/tags/multi-step-planning/', '255'),
            exact: true
          },
          {
            path: '/docs/tags/natural-language/',
            component: ComponentCreator('/docs/tags/natural-language/', '6f2'),
            exact: true
          },
          {
            path: '/docs/tags/nav-2/',
            component: ComponentCreator('/docs/tags/nav-2/', 'dfe'),
            exact: true
          },
          {
            path: '/docs/tags/navigation/',
            component: ComponentCreator('/docs/tags/navigation/', '812'),
            exact: true
          },
          {
            path: '/docs/tags/path-planning/',
            component: ComponentCreator('/docs/tags/path-planning/', '91d'),
            exact: true
          },
          {
            path: '/docs/tags/perception-pipelines/',
            component: ComponentCreator('/docs/tags/perception-pipelines/', '5ab'),
            exact: true
          },
          {
            path: '/docs/tags/perception/',
            component: ComponentCreator('/docs/tags/perception/', '806'),
            exact: true
          },
          {
            path: '/docs/tags/photorealistic-rendering/',
            component: ComponentCreator('/docs/tags/photorealistic-rendering/', 'c60'),
            exact: true
          },
          {
            path: '/docs/tags/physics-simulation/',
            component: ComponentCreator('/docs/tags/physics-simulation/', 'cbd'),
            exact: true
          },
          {
            path: '/docs/tags/quickstart/',
            component: ComponentCreator('/docs/tags/quickstart/', '74e'),
            exact: true
          },
          {
            path: '/docs/tags/robotics/',
            component: ComponentCreator('/docs/tags/robotics/', '00a'),
            exact: true
          },
          {
            path: '/docs/tags/ros-2-actions/',
            component: ComponentCreator('/docs/tags/ros-2-actions/', '238'),
            exact: true
          },
          {
            path: '/docs/tags/ros-integration/',
            component: ComponentCreator('/docs/tags/ros-integration/', '038'),
            exact: true
          },
          {
            path: '/docs/tags/setup/',
            component: ComponentCreator('/docs/tags/setup/', '4c7'),
            exact: true
          },
          {
            path: '/docs/tags/simulation/',
            component: ComponentCreator('/docs/tags/simulation/', '974'),
            exact: true
          },
          {
            path: '/docs/tags/speech-to-text/',
            component: ComponentCreator('/docs/tags/speech-to-text/', 'fe5'),
            exact: true
          },
          {
            path: '/docs/tags/synthetic-data/',
            component: ComponentCreator('/docs/tags/synthetic-data/', '523'),
            exact: true
          },
          {
            path: '/docs/tags/vision-language-action/',
            component: ComponentCreator('/docs/tags/vision-language-action/', '233'),
            exact: true
          },
          {
            path: '/docs/tags/vla-integration/',
            component: ComponentCreator('/docs/tags/vla-integration/', 'caa'),
            exact: true
          },
          {
            path: '/docs/tags/vla/',
            component: ComponentCreator('/docs/tags/vla/', 'ea7'),
            exact: true
          },
          {
            path: '/docs/tags/voice-control/',
            component: ComponentCreator('/docs/tags/voice-control/', 'da0'),
            exact: true
          },
          {
            path: '/docs/tags/voice-recognition/',
            component: ComponentCreator('/docs/tags/voice-recognition/', '035'),
            exact: true
          },
          {
            path: '/docs/tags/vslam/',
            component: ComponentCreator('/docs/tags/vslam/', '75e'),
            exact: true
          },
          {
            path: '/docs/tags/whisper/',
            component: ComponentCreator('/docs/tags/whisper/', 'f87'),
            exact: true
          },
          {
            path: '/docs/',
            component: ComponentCreator('/docs/', '3f7'),
            routes: [
              {
                path: '/docs/intro/',
                component: ComponentCreator('/docs/intro/', 'dbd'),
                exact: true
              },
              {
                path: '/docs/module-1/chapter-1-ros2-fundamentals/',
                component: ComponentCreator('/docs/module-1/chapter-1-ros2-fundamentals/', '832'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/chapter-2-python-agents/',
                component: ComponentCreator('/docs/module-1/chapter-2-python-agents/', '1c7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/chapter-3-urdf-essentials/',
                component: ComponentCreator('/docs/module-1/chapter-3-urdf-essentials/', '07d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/chapter-1-gazebo-basics/',
                component: ComponentCreator('/docs/module-2/chapter-1-gazebo-basics/', '58f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/chapter-2-unity-interaction/',
                component: ComponentCreator('/docs/module-2/chapter-2-unity-interaction/', '658'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/chapter-3-sensor-simulation/',
                component: ComponentCreator('/docs/module-2/chapter-3-sensor-simulation/', '87e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/intro/',
                component: ComponentCreator('/docs/module-2/intro/', 'aee'),
                exact: true
              },
              {
                path: '/docs/module-3/chapter-1-isaac-sim-essentials/',
                component: ComponentCreator('/docs/module-3/chapter-1-isaac-sim-essentials/', '4aa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3/chapter-2-isaac-ros-integration/',
                component: ComponentCreator('/docs/module-3/chapter-2-isaac-ros-integration/', '266'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3/chapter-3-nav2-humanoid-navigation/',
                component: ComponentCreator('/docs/module-3/chapter-3-nav2-humanoid-navigation/', '690'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3/intro/',
                component: ComponentCreator('/docs/module-3/intro/', 'd43'),
                exact: true
              },
              {
                path: '/docs/module-3/quickstart/',
                component: ComponentCreator('/docs/module-3/quickstart/', 'bf8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/chapter-1-voice-to-action/',
                component: ComponentCreator('/docs/module-4/chapter-1-voice-to-action/', 'dbe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/chapter-2-cognitive-planning/',
                component: ComponentCreator('/docs/module-4/chapter-2-cognitive-planning/', 'ded'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/chapter-3-autonomous-humanoid/',
                component: ComponentCreator('/docs/module-4/chapter-3-autonomous-humanoid/', '08d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/intro/',
                component: ComponentCreator('/docs/module-4/intro/', 'd36'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/quickstart/',
                component: ComponentCreator('/docs/module-4/quickstart/', '85b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/congratulations/',
                component: ComponentCreator('/docs/tutorial-basics/congratulations/', 'a11'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/create-a-blog-post/',
                component: ComponentCreator('/docs/tutorial-basics/create-a-blog-post/', '5af'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/create-a-document/',
                component: ComponentCreator('/docs/tutorial-basics/create-a-document/', 'fd1'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/create-a-page/',
                component: ComponentCreator('/docs/tutorial-basics/create-a-page/', '98f'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/deploy-your-site/',
                component: ComponentCreator('/docs/tutorial-basics/deploy-your-site/', 'af9'),
                exact: true
              },
              {
                path: '/docs/tutorial-basics/markdown-features/',
                component: ComponentCreator('/docs/tutorial-basics/markdown-features/', 'fdc'),
                exact: true
              },
              {
                path: '/docs/tutorial-extras/manage-docs-versions/',
                component: ComponentCreator('/docs/tutorial-extras/manage-docs-versions/', '22e'),
                exact: true
              },
              {
                path: '/docs/tutorial-extras/translate-your-site/',
                component: ComponentCreator('/docs/tutorial-extras/translate-your-site/', 'f0b'),
                exact: true
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
