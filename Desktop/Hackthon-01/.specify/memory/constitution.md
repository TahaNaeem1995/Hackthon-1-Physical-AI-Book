<!-- SYNC IMPACT REPORT:
Version change: N/A → 1.0.0
Added sections: All sections (new constitution)
Removed sections: None (first version)
Modified principles: None (new constitution)
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ✅ reviewed
- README.md ⚠ pending
Follow-up TODOs: None
-->

# AI-Native Book on Physical AI & Humanoid Robotics Constitution

## Core Principles

### AI-Native Content Generation
Content must be AI-generated or AI-refined only, with no plagiarism. All content originates from Claude Code and Spec-Kit Plus with proper attribution where applicable. Human oversight ensures technical accuracy while maintaining AI-native generation principles.

### Technical Accuracy in AI and Robotics
All content related to Physical AI and Humanoid Robotics must maintain high technical accuracy. Content undergoes verification for correctness in robotics concepts, AI algorithms, and physical systems integration. Technical claims must be substantiated or clearly marked as speculative.

### Modular and Clear Documentation (NON-NEGOTIABLE)
Chapters and sections must be modular, with clear headings and minimal verbosity. Each section should be self-contained enough to be understood independently while contributing to the overall narrative. Structure follows Docusaurus-compatible Markdown/MDX format.

### Reproducible via Specs and Prompts
All content generation must be reproducible through specifications and prompts. Every chapter, section, and major content piece must be tied to specific prompts and specifications that can recreate the content. This ensures version control and collaborative refinement.

### Interactive Learning Through Embedded Systems
The book must incorporate interactive elements, particularly the embedded RAG chatbot. User engagement features should enhance learning outcomes. Content should be structured to support question-answering and interactive exploration of concepts.

### Deployment-First Mindset
Development follows a deployment-first approach with Docusaurus on GitHub Pages. All content and features must be deployable and functional in the production environment. Local development mirrors production as closely as possible.

## Technical Standards

### Content Format Requirements
- Primary format: Markdown / MDX (Docusaurus compatible)
- Secondary format: Structured for RAG indexing and retrieval
- File structure: Organized in chapters with clear navigation hierarchy
- Metadata: Each document includes proper frontmatter for Docusaurus

### Technical Stack Compliance
- Authoring: Claude Code + Spec-Kit Plus (primary)
- Frontend: Docusaurus framework (GitHub Pages deployment)
- Backend services: FastAPI for RAG chatbot
- Vector Database: Qdrant Cloud (Free tier)
- Metadata management: Neon Serverless Postgres
- All components must integrate seamlessly with the overall architecture

### Quality Assurance
- Content accuracy verification for robotics and AI concepts
- Cross-referencing between related topics and chapters
- Consistent terminology and concept definitions
- Regular validation of RAG system performance and accuracy

## Development Workflow

### Content Creation Process
- Specifications drive content creation via Claude Code
- Each chapter begins with a clear specification document
- Content undergoes AI refinement cycles with human oversight
- Peer review process for technical accuracy validation

### Integration Requirements
- New content must integrate with existing RAG system
- Navigation and cross-linking maintained across all sections
- Embedded chatbot functionality tested with new content
- Deployment pipeline validated with each content addition

### Quality Gates
- Technical accuracy verification for all AI/robotics content
- RAG system responds correctly to queries about new content
- Docusaurus site builds without errors
- Cross-reference integrity maintained

## Governance

This constitution governs all content creation, technical implementation, deployment, and chatbot behavior for the AI-Native Book on Physical AI & Humanoid Robotics project. All contributors must adhere to these principles. Amendments to this constitution require explicit documentation of changes, impact assessment, and approval from project maintainers. All development activities must reference this constitution for compliance verification.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17