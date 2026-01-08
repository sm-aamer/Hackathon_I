# Quickstart Guide: Digital Twin Simulation Module

## Overview
This quickstart guide will help you understand and contribute to Module-2: The Digital Twin (Gazebo & Unity) of the Physical AI & Humanoid Robotics book.

## Module Structure

Module-2 covers simulation technologies for humanoid robotics with three main chapters:

1. **Gazebo Simulation Basics** - Physics, gravity, and collision simulation
2. **Unity for Human-Robot Interaction** - Environments and rendering
3. **Sensor Simulation** - LiDAR, depth cameras, and IMUs

## Adding New Content

To add a new simulation chapter:

1. Create a new Markdown file in the `my-website/docs/module-2/` directory
2. Add the content with proper frontmatter:
   ```markdown
   ---
   id: chapter-1-gazebo-basics
   sidebar_label: Gazebo Simulation Basics
   title: Chapter 1 - Gazebo Simulation Basics
   ---
   ```

3. Update `sidebars.js` to include the new chapter in the navigation

## Content Guidelines for Simulation Topics

### For Gazebo Content
- Include version information for Gazebo
- Provide configuration file examples
- Add visual aids (screenshots, diagrams) where helpful
- Include troubleshooting tips for common issues

### For Unity Content
- Specify Unity version and compatible packages
- Include step-by-step visual guides
- Provide project structure recommendations
- Add performance optimization tips

### For Sensor Simulation
- Detail sensor specifications and parameters
- Include sample data outputs
- Provide comparison between virtual and real sensor data
- Add calibration guidance

## Technical Requirements

### System Requirements for Simulation Examples
- **Gazebo**: Compatible with ROS 2 Humble Hawksbill or later
- **Unity**: Version 2021.3 LTS or later recommended
- **Hardware**: GPU with OpenGL 3.3+ support for rendering

### Documentation Standards
- Use consistent terminology throughout all chapters
- Include code snippets with proper syntax highlighting
- Add cross-references between related concepts
- Ensure all examples are simulation-ready and testable

## Integration with Existing Content

Module-2 is designed to complement Module-1 (ROS 2 fundamentals) while maintaining independence. Students can learn simulation concepts without deep ROS 2 knowledge, but integration points are highlighted where beneficial.

## Building and Testing

To build the documentation site with Module-2:
```bash
cd my-website
npm run build
```

To run locally and preview Module-2 content:
```bash
cd my-website
npm start
```

Navigate to `/docs/module-2/chapter-1-gazebo-basics` to access the module content.

## Contribution Workflow

1. Create a new branch for your simulation content changes
2. Add or modify content in the `my-website/docs/module-2/` directory
3. Update navigation in `sidebars.js` if adding new sections
4. Test locally to ensure proper rendering
5. Submit a pull request with clear description of the simulation concepts covered