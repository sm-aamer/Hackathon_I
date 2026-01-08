---
id: 1
title: Update Project Constitution
stage: constitution
date: 2026-01-07
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution", "spec-driven", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - ".specify/memory/constitution.md"
tests:
 - none
---

## Prompt

Project: Book — Physical AI & Humanoid Robotics

Core Principles:

Spec-driven creation using Claude Code + Spec-Kit Plus

Technical accuracy, coherence, and reproducibility

RAG-ready structure with clean modular sections

Standards:

Built in Docusaurus and deployed on GitHub Pages

All content internally consistent and verifiable

Chatbot integration using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant

Requirements:

Generate full book via specs

Publish site with embedded RAG chatbot

Chatbot must answer from book content and user-selected text

Constraints:

Must run on free-tier Qdrant + Neon

Fully functional UI on GitHub Pages

Success Criteria:

Book deploys successfully

Chatbot works reliably

Spec-driven outputs remain consistent across the project.

## Response snapshot

Updated the project constitution with specific principles for the Book — Physical AI & Humanoid Robotics project, including spec-driven creation, technical accuracy, RAG-ready architecture, standards compliance, resource optimization, and quality & reliability.

## Outcome

- ✅ Impact: Project constitution updated with domain-specific principles
- 🧪 Tests: none
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: Continue with spec, plan, and task generation
- 🧠 Reflection: Clear project principles help guide future development decisions

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A