# Data Model: Module 4: Vision-Language-Action (VLA)

**Feature**: 1-vla-robotics
**Created**: 2026-01-08
**Modeler**: Claude Sonnet 4.5

## Content Entities

### 1. VoiceCommand
Represents a natural language command input from a user that needs to be processed and executed.

**Attributes**:
- id: Unique identifier for the command
- text: Original spoken text converted to string
- timestamp: When the command was received
- confidence: Confidence score from speech recognition (0.0-1.0)
- userId: Identifier of the user who issued the command
- sessionId: Session context for multi-turn interactions

**Relationships**:
- 1 VoiceCommand → 1 LLMInterpretation
- 1 VoiceCommand → * ActionPlan (via interpretation)

### 2. LLMInterpretation
Structured representation of user intent derived from natural language using cognitive models.

**Attributes**:
- id: Unique identifier for the interpretation
- commandId: Reference to the original voice command
- intent: Classified intent (e.g., "NAVIGATE", "MANIPULATE", "PERCEIVE")
- parameters: JSON object containing action parameters
- confidence: Confidence score from LLM interpretation (0.0-1.0)
- timestamp: When the interpretation was generated

**Relationships**:
- 1 LLMInterpretation → * ActionPlan
- 1 VoiceCommand ← 1 LLMInterpretation

### 3. ActionPlan
Sequence of ROS 2 actions that implement the user's request.

**Attributes**:
- id: Unique identifier for the action plan
- interpretationId: Reference to the LLM interpretation
- sequence: Array of action steps in execution order
- status: Current status (PENDING, EXECUTING, COMPLETED, FAILED)
- createdAt: Timestamp when plan was created
- updatedAt: Timestamp when plan was last updated

**Relationships**:
- 1 ActionPlan → * ActionStep
- 1 LLMInterpretation ← 1 ActionPlan

### 4. ActionStep
Individual action within an action plan.

**Attributes**:
- id: Unique identifier for the action step
- planId: Reference to the parent action plan
- actionType: Type of action (NAVIGATION, MANIPULATION, PERCEPTION, etc.)
- parameters: Specific parameters for the action
- order: Execution order within the plan
- status: Status of this step (PENDING, EXECUTING, COMPLETED, FAILED)
- result: Result data from execution (optional)

**Relationships**:
- 1 ActionStep → 1 ActionPlan (inverse)
- 1 ActionStep → 1 ExecutionState (optional)

### 5. ExecutionState
Current status of ongoing command execution, including progress and any errors encountered.

**Attributes**:
- id: Unique identifier for the execution state
- stepId: Reference to the action step being executed
- robotState: Current state of the robot (position, battery, etc.)
- progress: Progress percentage (0-100)
- statusMessage: Human-readable status message
- errors: Array of error messages (if any)
- startTime: When execution started
- endTime: When execution completed (if completed)

**Relationships**:
- 1 ExecutionState → 1 ActionStep (inverse)

### 6. EnvironmentalModel
Representation of the robot's surroundings including objects, obstacles, and navigable areas.

**Attributes**:
- id: Unique identifier for the environmental model
- timestamp: When the model was created/updated
- mapData: Serialized map representation
- objects: Array of recognized objects with positions
- obstacles: Array of obstacles with positions
- navigableAreas: Regions where the robot can move
- confidence: Overall confidence in the environmental model (0.0-1.0)

**Relationships**:
- 1 EnvironmentalModel → * ActionStep (used during execution)

## Content Structure for Documentation

### 7. VLAChapter
Documentation chapter in the VLA module.

**Attributes**:
- id: Unique identifier for the chapter
- title: Chapter title
- slug: URL-friendly identifier
- content: Markdown content of the chapter
- order: Chapter order in the module (1, 2, 3)
- prerequisites: Array of prerequisite concepts
- objectives: Learning objectives for the chapter
- duration: Estimated completion time in minutes

**Relationships**:
- 1 VLAChapter → * VLALesson (contains lessons)
- 1 VLAChapter → * VLAExercise (contains exercises)

### 8. VLALesson
Individual lesson within a VLA chapter.

**Attributes**:
- id: Unique identifier for the lesson
- chapterId: Reference to the parent chapter
- title: Lesson title
- content: Markdown content of the lesson
- order: Lesson order within the chapter
- duration: Estimated completion time in minutes
- type: Type of lesson (CONCEPTUAL, PRACTICAL, DEMONSTRATION)

**Relationships**:
- 1 VLALesson → 1 VLAChapter (inverse)
- 1 VLALesson → * VLAExercise (contains exercises)

### 9. VLAExercise
Hands-on exercise for students to practice VLA concepts.

**Attributes**:
- id: Unique identifier for the exercise
- lessonId: Reference to the parent lesson
- title: Exercise title
- description: Detailed exercise description
- steps: Array of step-by-step instructions
- expectedOutcome: What students should achieve
- difficulty: Difficulty level (BEGINNER, INTERMEDIATE, ADVANCED)
- estimatedTime: Time needed to complete the exercise

**Relationships**:
- 1 VLAExercise → 1 VLALesson (inverse)

## Validation Rules

### Voice Command Validation
- Text must be between 5 and 500 characters
- Confidence score must be between 0.0 and 1.0
- Command must belong to a valid user session

### LLM Interpretation Validation
- Intent must be one of predefined values: NAVIGATE, MANIPULATE, PERCEIVE, QUERY
- Parameters must conform to expected schema for the intent
- Confidence score must be between 0.0 and 1.0

### Action Plan Validation
- Must contain at least one action step
- All action steps must have valid action types
- Action sequence must be logically consistent

### Content Structure Validation
- Chapters must have unique titles and slugs
- Lessons must be ordered sequentially (1, 2, 3, etc.)
- Exercises must reference valid lessons
- Prerequisites must reference existing content entities