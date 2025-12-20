---
description: Project Constitution - AI-Native Textbook on Physical AI & Humanoid Robotics
---

# 🤖 AI-Native Textbook: Physical AI & Humanoid Robotics

## Project Constitution

This document serves as the authoritative guide for developing the AI-Native Textbook focused on Physical AI and Humanoid Robotics, designed for the Panaversity Hackathon.

---

## 🎯 Core Principles

### 1. Human-Agent-Robot Symbiosis
- Focus on the **partnership** between people, software agents, and physical robots
- Emphasize collaborative intelligence over pure automation
- Design content that bridges human intuition with machine capability

### 2. AI-Native Pedagogy
- Content designed specifically for **AI-assisted learning**
- Leverage Claude Code & Agents for interactive learning experiences
- Structure enables future AI agents to read, parse, and learn from content

### 3. Practical Rigor
- Balance theoretical foundations with **actionable, deployable code examples**
- Every concept accompanied by working implementations
- Real-world applicability in robotics and physical AI systems

### 4. Future-Ready Skills
- Curriculum aligned with **"Future of Work"** demands
- Focus on skills that will remain relevant as AI evolves
- Emphasis on human-AI collaboration capabilities

---

## 📐 Technical Standards

### Technology Stack
| Component | Technology | Purpose |
|-----------|------------|---------|
| Framework | **Docusaurus** | Static site generation & documentation |
| Scaffolding | **Spec-Kit Plus** | Project structure & component templates |
| Content Generation | **Claude Code** | AI-assisted content creation |
| Hosting | **GitHub Pages** | Static site deployment |
| Format | **Markdown** | Content authoring |

### Output Format
- **Markdown-based documentation** deployed to GitHub Pages
- MDX support for interactive components
- Code blocks with syntax highlighting and copy functionality

### Content Structure
- **Modular chapters** with clear learning objectives
- **Interactive code blocks** for hands-on learning
- **Progressive complexity** from fundamentals to advanced topics
- Each section includes: Learning Objectives → Theory → Code Examples → Exercises

---

## 👥 Target Audience

### Primary Audience
- **Technical learners** with backgrounds in:
  - O/A Level students (STEM focus)
  - Engineering students and professionals
  - Medical technology enthusiasts
  - AI/ML practitioners transitioning to physical AI

### Prerequisites
- Basic programming knowledge (Python preferred)
- Understanding of fundamental computer science concepts
- Interest in robotics and physical computing

---

## 🚧 Constraints

### Tooling Requirements
- **MUST** use Spec-Kit Plus for scaffolding
- **MUST** use Claude Code for content generation
- **MUST** follow Docusaurus conventions

### Platform Requirements
- Static site hosting via **GitHub Pages**
- No server-side processing dependencies
- PWA-ready for offline learning

### Content Scope
- Intersection of **physical hardware control** and **AI agent logic**
- Focus on humanoid robotics and embodied AI
- Practical implementations over theoretical abstractions

---

## ✅ Success Criteria

### 1. Deployment
- ✅ Zero-error build process
- ✅ Accessible via **public GitHub Pages URL**
- ✅ Responsive design across devices
- ✅ < 3 second initial page load

### 2. Completeness
- ✅ Full course curriculum covered in Docusaurus structure
- ✅ All core topics with code examples
- ✅ Navigation and search functionality
- ✅ Clear learning pathways

### 3. Scalability
- ✅ Architecture allows for **future AI-agent integration**
- ✅ Machine-readable content structure
- ✅ Extensible component system
- ✅ Version-controlled content

---

## 📚 Curriculum Overview

### Part 1: Foundations
1. Introduction to Physical AI
2. Humanoid Robotics Fundamentals
3. AI Agent Architecture

### Part 2: Core Technologies
4. Sensor Integration & Perception
5. Motor Control & Actuation
6. Computer Vision for Robotics
7. Natural Language Interfaces

### Part 3: Intelligence Layer
8. Decision Making & Planning
9. Learning from Demonstration
10. Multi-Agent Coordination

### Part 4: Integration & Deployment
11. System Integration Patterns
12. Safety & Ethics in Physical AI
13. Deployment & Operations

---

## 🔧 Development Workflow

### Step 1: Initialize Project
```bash
// turbo
npx create-docusaurus@latest ./ classic --typescript
```

### Step 2: Install Dependencies
```bash
// turbo
npm install
```

### Step 3: Local Development
```bash
npm run start
```

### Step 4: Build for Production
```bash
// turbo
npm run build
```

### Step 5: Deploy to GitHub Pages
```bash
npm run deploy
```

---

## 📁 Project Structure

```
ai-native-book/
├── .agent/
│   └── workflows/
│       └── sp.constitution.md    # This file
├── docs/
│   ├── intro.md                  # Course introduction
│   ├── part-1-foundations/       # Foundation chapters
│   ├── part-2-core-tech/         # Core technology chapters
│   ├── part-3-intelligence/      # Intelligence layer chapters
│   └── part-4-integration/       # Integration chapters
├── src/
│   ├── components/               # Custom React components
│   ├── css/                      # Custom styling
│   └── pages/                    # Custom pages
├── static/
│   └── img/                      # Static assets
├── docusaurus.config.js          # Site configuration
├── sidebars.js                   # Navigation structure
└── package.json                  # Dependencies
```

---

## 🎨 Design Guidelines

### Visual Identity
- **Primary Color**: Deep Blue (#1a73e8) - Trust & Technology
- **Accent Color**: Electric Cyan (#00d4ff) - Innovation & Future
- **Dark Mode**: Fully supported with optimized palette

### Typography
- **Headings**: Inter or Outfit (modern, technical)
- **Body**: System fonts for performance
- **Code**: JetBrains Mono or Fira Code

### UX Principles
- Clear visual hierarchy
- Consistent navigation patterns
- Mobile-first responsive design
- Accessible to WCAG 2.1 AA standards

---

*Last Updated: December 2024*
*Version: 1.0.0*
