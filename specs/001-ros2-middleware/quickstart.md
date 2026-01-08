# Quickstart Guide: Docusaurus ROS 2 Documentation Site

## Getting Started

This quickstart guide will help you set up and run the Docusaurus-based documentation site for the "Physical AI & Humanoid Robotics" book.

### Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git

### Installation Steps

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd my-website
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm start
   # or
   yarn start
   ```

   This command starts a local development server and opens the website in your browser at `http://localhost:3000`.

4. **Build for production**
   ```bash
   npm run build
   # or
   yarn build
   ```

   This command generates static content in the `build` directory, which can be served using any static hosting service.

### Project Structure

```
my-website/
├── blog/                    # Blog posts (if needed)
├── docs/                    # Documentation files
│   ├── module-1/            # Module 1: The Robotic Nervous System (ROS 2)
│   │   ├── chapter-1-ros2-fundamentals.md    # Chapter 1: ROS 2 Fundamentals
│   │   ├── chapter-2-python-agents.md        # Chapter 2: Python Agents + rclpy Integration
│   │   └── chapter-3-urdf-essentials.md      # Chapter 3: Humanoid Robot URDF Essentials
│   ├── module-2/            # Module 2: Future modules structure
│   └── ...
├── src/
│   ├── components/          # Custom React components
│   ├── pages/               # Custom pages
│   └── css/                 # Custom styles
├── static/                  # Static files (images, etc.)
├── docusaurus.config.js     # Docusaurus configuration
├── sidebars.js              # Navigation sidebar configuration
└── package.json             # Node.js dependencies
```

### Adding New Content

To add a new chapter:

1. Create a new Markdown file in the appropriate module directory
2. Add the content with proper frontmatter:
   ```markdown
   ---
   id: chapter-title
   sidebar_label: Chapter Title
   title: Chapter Title
   ---
   ```

3. Update `sidebars.js` to include the new chapter in the navigation

### Configuration

The site configuration is managed in `docusaurus.config.js`:
- Site metadata (title, tagline, URL)
- Theme configuration
- Plugin configuration
- Custom components

The navigation structure is defined in `sidebars.js`:
- Sidebar organization by module
- Chapter ordering
- Category grouping

### Deployment

The site is designed for GitHub Pages deployment:
1. Build the site: `npm run build`
2. The `build` directory contains the static assets ready for deployment
3. Configure GitHub Pages to serve from the `build` directory or use GitHub Actions for automated deployment