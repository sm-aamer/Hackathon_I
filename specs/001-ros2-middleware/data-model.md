# Data Model: Docusaurus ROS 2 Documentation Site

## Overview
This document defines the data structures and entities for the Docusaurus-based documentation site, focusing on the content organization and metadata needed for the ROS 2 documentation modules.

## Content Entities

### Module
- **name**: String - The module name (e.g., "Module 1: The Robotic Nervous System")
- **description**: String - Brief description of the module
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
- **metadata**: Object - Additional metadata
  - **sidebar_label**: String - Label for sidebar navigation
  - **position**: Number - Position in module order
  - **tags**: Array<String> - Content tags for search and filtering

### Content Block
- **type**: Enum - Type of content block (text, code, diagram, exercise, example)
- **content**: String - The content of the block
- **chapter_id**: String - Reference to parent chapter
- **position**: Number - Position within the chapter

### Navigation Item
- **id**: String - Unique identifier
- **label**: String - Display label
- **href**: String - URL path or external link
- **type**: Enum - Type of navigation item (doc, link, category)
- **items**: Array<Navigation Item> - Child navigation items (for categories)

## Relationships
- Module 1-* Chapter
- Chapter 1-* Content Block
- Navigation Item 1-* Navigation Item (for hierarchical structure)

## Validation Rules
- Module names must be unique within the site
- Chapter slugs must be unique within their module
- Content blocks must have valid positions within chapters
- Navigation items must have valid references to documents or valid external URLs

## State Transitions
- Content draft → review → published
- Navigation item enabled/disabled based on content availability