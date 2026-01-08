# Data Model: Digital Twin Simulation Module

## Overview
This document defines the content structure and entities for Module-2: The Digital Twin (Gazebo & Unity), focusing on simulation content organization and metadata needed for the documentation.

## Content Entities

### Module
- **name**: String - The module name ("Module 2: The Digital Twin (Gazebo & Unity)")
- **description**: String - Brief description of the simulation module
- **chapters**: Array<Chapter> - List of chapters in the module
- **metadata**: Object - Additional metadata for navigation and search
  - **sidebar_label**: String - Label for sidebar navigation
  - **position**: Number - Position in navigation order
  - **tags**: Array<String> - Content tags for search and filtering

### Chapter
- **title**: String - Chapter title
- **slug**: String - URL-friendly identifier
- **content**: String - The chapter content in Markdown format
- **module_id**: String - Reference to parent module
- **prerequisites**: Array<String> - Prerequisite knowledge required
- **learning_objectives**: Array<String> - What the student will learn
- **simulation_tools**: Array<String> - Tools covered in the chapter (e.g., "Gazebo", "Unity", "LiDAR")
- **metadata**: Object - Additional metadata
  - **sidebar_label**: String - Label for sidebar navigation
  - **position**: Number - Position in module order
  - **tags**: Array<String> - Content tags for search and filtering

### Simulation Concept
- **name**: String - Name of the simulation concept (e.g., "Physics Engine", "Collision Detection")
- **definition**: String - Clear definition of the concept
- **application**: String - How the concept applies in simulation
- **chapter_id**: String - Reference to parent chapter
- **examples**: Array<String> - Practical examples of the concept
- **best_practices**: Array<String> - Recommended practices for implementation

### Code Example
- **type**: Enum - Type of code example (configuration, script, command, snippet)
- **content**: String - The actual code content
- **language**: String - Programming language or configuration format
- **chapter_id**: String - Reference to parent chapter
- **purpose**: String - What the code example demonstrates
- **simulation_tool**: String - Which simulation tool the code relates to

### Asset
- **type**: Enum - Type of asset (image, video, diagram, sample_file, configuration)
- **filename**: String - Name of the asset file
- **description**: String - What the asset illustrates or provides
- **chapter_id**: String - Reference to parent chapter
- **usage_context**: String - How the asset is used in the chapter

## Relationships
- Module 1-* Chapter
- Chapter 1-* Simulation Concept
- Chapter 1-* Code Example
- Chapter 1-* Asset
- Simulation Concept 1-* Code Example (many-to-many relationship)

## Validation Rules
- Module names must be unique within the site
- Chapter slugs must be unique within their module
- Simulation concepts must have valid definitions
- Code examples must have specified languages
- Assets must have valid file references

## State Transitions
- Content draft → review → published
- Navigation item enabled/disabled based on content availability
- Asset status: uploaded → processed → referenced → archived