<!-- Sync Impact Report -->
<!--
Version change: 0.0.0 → 1.0.0
Modified principles:
  - [PROJECT_NAME] → Physical AI & Humanoid Robotics
  - [PRINCIPLE_1_NAME] → Docusaurus-First Documentation
  - [PRINCIPLE_2_NAME] → Academic and Concise Tone
  - [PRINCIPLE_3_NAME] → Professional and Accessible Language
  - [PRINCIPLE_4_NAME] → Content Accuracy and Technical Rigor
  - [PRINCIPLE_5_NAME] → Maintainability and Scalability
  - [PRINCIPLE_6_NAME] → Accessibility and Inclusivity
Added sections: None
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/*.md: ✅ updated
  - README.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Docusaurus-First Documentation
All book content and documentation will be managed and generated using Docusaurus. Use standard Markdown features, Admonitions (`:::tip`, `:::info`), and Code Blocks to ensure a high-quality reading experience.

### II. Academic and Concise Tone
Content must maintain an academic, concise, and implementation-focused tone suitable for a technical university textbook. Explain the "Why" before the "How."


### IV. Content Accuracy and Technical Rigor
All technical claims must be accurate. If a specific version is not mentioned, assume the latest stable release (e.g., ROS 2 Humble/Jazzy). Do not hallucinate commands; use real API references.

### V. Accessibility and Inclusivity
The final material must be accessible to beginners. Jargon should be explained upon first use.

## Technical Stack Requirements (MANDATORY)

While the book is built with Docusaurus, the **Subject Matter** must strictly adhere to the following technologies. Do not use alternatives:

1.  **Middleware:** **ROS 2 (Robot Operating System 2)**. Use **Python (`rclpy`)** for all scripting. Do NOT use ROS 1 or C++ unless necessary.
2.  **Simulation:** **Gazebo** for physics simulation and **Unity** for high-fidelity rendering.
3.  **AI & Perception:** **NVIDIA Isaac Sim** and **Isaac SDK** for the AI-Robot brain.
4.  **VLA & NLP:** **OpenAI Whisper** for voice commands and **LLMs** for cognitive planning.
5.  **Robot Format:** Use **URDF** (Unified Robot Description Format) for humanoids.

## Development Workflow


2.  **Generate Sequentially:** Create chapters in order (Chapter 1 -> Chapter 2).
3.  **Code Validation:** Ensure all Python and XML (URDF) code examples are syntactically correct and commented.

## Governance
This Constitution outlines the foundational principles and mandatory rules for the "Physical AI & Humanoid Robotics" project. It supersedes any conflicting default AI behaviors.

**Version**: 1.2.0 (Hackathon Edition) | **Ratified**: 2025-12-04 | **Status**: ACTIVE