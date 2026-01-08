<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: [PRINCIPLE_1_NAME] → Spec-Driven Creation, [PRINCIPLE_2_NAME] → Technical Accuracy & Coherence, [PRINCIPLE_3_NAME] → RAG-Ready Architecture, [PRINCIPLE_4_NAME] → Standards Compliance, [PRINCIPLE_5_NAME] → Resource Optimization, [PRINCIPLE_6_NAME] → Quality & Reliability
- Added sections: None
- Removed sections: None
- Templates requiring updates: ✅ All templates checked and aligned
- Follow-up TODOs: None
-->

# Book — Physical AI & Humanoid Robotics Constitution

## Core Principles

### Spec-Driven Creation
Spec-driven creation using Claude Code + Spec-Kit Plus; All features and content must originate from and remain consistent with formal specifications; Clear traceability between specs and implementations required.

### Technical Accuracy & Coherence
Technical accuracy, coherence, and reproducibility of all content; All technical claims must be verifiable and factually correct; Consistent terminology and concepts across all book sections.

### RAG-Ready Architecture
RAG-ready structure with clean modular sections for AI retrieval; Content organized for efficient vector search and semantic understanding; Modular, well-defined sections that work independently yet coherently.

### Standards Compliance
Built in Docusaurus and deployed on GitHub Pages with embedded chatbot; All content internally consistent and verifiable; Chatbot integration using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant.

### Resource Optimization
Must run on free-tier Qdrant + Neon while maintaining performance; Efficient resource utilization and cost-conscious architecture; Optimization for free-tier constraints without sacrificing functionality.

### Quality & Reliability
Book deploys successfully with reliable chatbot functionality; Spec-driven outputs remain consistent across the project; All features must be tested and validated before release.

## Additional Requirements

All content generation must follow spec-driven methodology; The system must generate full book via specs; The published site must include embedded RAG chatbot that answers from book content and user-selected text; The UI must be fully functional on GitHub Pages.

## Development Workflow

The project follows Spec-Kit Plus methodology with distinct stages: spec, plan, tasks, implementation; All changes must be small, testable, and reference code precisely; Prompt History Records (PHRs) must be created for every user interaction; Architectural Decision Records (ADRs) must document significant decisions.

## Governance

This constitution governs all development decisions and supersedes any conflicting practices; Amendments require documentation of rationale and impact assessment; All PRs and reviews must verify compliance with these principles; Versioning follows semantic versioning with MAJOR.MINOR.PATCH format.

**Version**: 1.1.0 | **Ratified**: 2026-01-07 | **Last Amended**: 2026-01-07