# Quickstart Guide: Physical AI Documentation Site

## Prerequisites

- Node.js (version 18.0 or higher)
- npm or yarn package manager
- Git
- GitHub account (for deployment)

## Installation Steps

1. **Install Docusaurus globally**:
   ```bash
   npm install -g @docusaurus/init
   ```

2. **Create a new Docusaurus project**:
   ```bash
   npx @docusaurus/init@latest init docs classic
   ```

3. **Navigate to the project directory**:
   ```bash
   cd docs
   ```

4. **Install dependencies**:
   ```bash
   npm install
   ```

## Configuration

1. **Update `docusaurus.config.js`** with GitHub Pages settings:
   ```javascript
   module.exports = {
     title: 'AI-Native Book on Physical AI & Humanoid Robotics',
     tagline: 'Foundations of Physical AI and Humanoid Robotics',
     url: 'https://<your-username>.github.io',
     baseUrl: '/ai-physical-ai-book/',
     organizationName: '<your-username>',
     projectName: 'ai-physical-ai-book',
     deploymentBranch: 'gh-pages',
     trailingSlash: false,
     onBrokenLinks: 'throw',
     onBrokenMarkdownLinks: 'warn',
     favicon: 'img/favicon.ico',
     // ... rest of configuration
   };
   ```

2. **Set up sidebar navigation** in `sidebars.js`:
   ```javascript
   module.exports = {
     tutorialSidebar: [
       'intro',
       {
         type: 'category',
         label: 'Module 1 - Foundations of Physical AI',
         items: [
           'module-1/module-1',
           'module-1/chapter-1',
           'module-1/chapter-2',
           'module-1/chapter-3',
         ],
       },
     ],
   };
   ```

## Creating Content

1. **Create module directory**:
   ```bash
   mkdir docs/module-1
   ```

2. **Add module overview file** (`docs/module-1/module-1.md`):
   ```markdown
   ---
   sidebar_position: 1
   ---

   # Module 1: Foundations of Physical AI

   This module establishes core understanding of Physical AI and humanoid robotics for beginner to intermediate AI/software learners.
   ```

3. **Add chapter files**:
   - `docs/module-1/chapter-1.md` - What Is Physical AI
   - `docs/module-1/chapter-2.md` - Evolution of Humanoid Robotics
   - `docs/module-1/chapter-3.md` - Core Components of Physical AI Systems

## Local Development

1. **Start development server**:
   ```bash
   npm start
   ```

2. **Open browser** to `http://localhost:3000` to view the documentation

## Deployment to GitHub Pages

1. **Build the static files**:
   ```bash
   npm run build
   ```

2. **Deploy to GitHub Pages**:
   ```bash
   GIT_USER=<Your GitHub username> npm run deploy
   ```

## File Structure

```
docs/
├── blog/              # Blog posts (optional)
├── docs/              # Documentation files
│   ├── module-1/
│   │   ├── module-1.md
│   │   ├── chapter-1.md
│   │   ├── chapter-2.md
│   │   └── chapter-3.md
│   └── ...
├── src/
│   ├── components/    # Custom React components
│   ├── css/          # Custom styles
│   └── pages/        # Custom pages
├── static/           # Static assets
├── docusaurus.config.js
├── package.json
└── sidebars.js
```