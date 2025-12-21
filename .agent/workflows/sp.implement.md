---
description: Implementation Workflow - Execute Project Tasks Step by Step
---

# 🛠️ Implementation Workflow: Execute Project Tasks

This workflow provides step-by-step instructions for implementing the remaining project tasks.

// turbo-all

---

## 🚀 Quick Start

```bash
# Ensure dev server is running
npm run start
```

---

## TASK-001: Create Downloadable Code Examples

### Step 1: Create directory structure

```bash
mkdir -p static/code-examples/module-1
mkdir -p static/code-examples/module-2
mkdir -p static/code-examples/module-3
mkdir -p static/code-examples/module-4
```

### Step 2: Create Module 1 code files

Create these files in `static/code-examples/module-1/`:

1. `hello_robot.py` - Basic ROS 2 node
2. `bipedal_robot.urdf` - Humanoid robot description
3. `robot_launch.py` - Launch file

### Step 3: Create Module 2 code files

Create in `static/code-examples/module-2/`:

1. `warehouse_world.sdf` - Gazebo world file
2. `physics_config.yaml` - Physics settings
3. `sensor_bridge.py` - ROS-Gazebo sensor bridge

### Step 4: Create Module 3 code files

Create in `static/code-examples/module-3/`:

1. `slam_node.py` - Visual SLAM implementation
2. `nav2_params.yaml` - Navigation parameters
3. `replicator_script.py` - Isaac Replicator for synthetic data

### Step 5: Create Module 4 code files

Create in `static/code-examples/module-4/`:

1. `whisper_ros_node.py` - Speech recognition node
2. `llm_task_planner.py` - LLM-based planner
3. `autonomous_humanoid.py` - Complete VLA system

### Step 6: Add download links to docs

For each chapter, add at the end:

```markdown
## 📥 Downloads

- [Download Code Examples](pathname:///code-examples/module-X/filename.py)
```

---

## TASK-002: Configure GitHub Actions

### Step 1: Create workflow directory

```bash
mkdir -p .github/workflows
```

### Step 2: Create deploy.yml

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4
        
      - name: Setup Node
        uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm
          
      - name: Install dependencies
        run: npm ci
        
      - name: Build website
        run: npm run build
        
      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: ./build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### Step 3: Verify docusaurus.config.ts

Ensure these settings are correct:

```typescript
const config: Config = {
  url: 'https://YOUR_USERNAME.github.io',
  baseUrl: '/ai-native-book/',
  organizationName: 'YOUR_USERNAME',
  projectName: 'ai-native-book',
  trailingSlash: false,
};
```

### Step 4: Push and verify

```bash
git add .
git commit -m "Add GitHub Actions deployment"
git push origin main
```

Check Actions tab in GitHub repository for deployment status.

---

## TASK-003: Add Exercise Solutions

### Step 1: Create solutions directory

```bash
mkdir -p static/solutions/module-1
mkdir -p static/solutions/module-2
mkdir -p static/solutions/module-3
mkdir -p static/solutions/module-4
```

### Step 2: Create solution files

For each exercise in the textbook, create a corresponding solution file.

### Step 3: Add solution component

Create `src/components/Solution.tsx`:

```tsx
import React, { useState } from 'react';

interface SolutionProps {
  children: React.ReactNode;
}

export default function Solution({ children }: SolutionProps) {
  const [show, setShow] = useState(false);
  
  return (
    <div className="solution-wrapper">
      <button 
        onClick={() => setShow(!show)}
        className="solution-toggle"
      >
        {show ? '🙈 Hide Solution' : '👀 Show Solution'}
      </button>
      {show && <div className="solution-content">{children}</div>}
    </div>
  );
}
```

### Step 4: Add solution styles

Add to `src/css/custom.css`:

```css
.solution-wrapper {
  margin: 1rem 0;
  padding: 1rem;
  border: 1px dashed var(--ifm-color-primary);
  border-radius: 8px;
}

.solution-toggle {
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  cursor: pointer;
  font-weight: bold;
}

.solution-toggle:hover {
  background: var(--ifm-color-primary-dark);
}

.solution-content {
  margin-top: 1rem;
  padding: 1rem;
  background: var(--ifm-background-surface-color);
  border-radius: 4px;
}
```

---

## TASK-004: Enhance Homepage

### Step 1: Update src/pages/index.tsx

Add hero section with gradient, feature cards, and CTA buttons.

### Step 2: Update src/pages/index.module.css

Add modern animations and responsive grid.

### Step 3: Add module preview cards

Create visual cards linking to each module.

---

## TASK-005: Add PWA Support

### Step 1: Install plugin

```bash
npm install @docusaurus/plugin-pwa
```

### Step 2: Create PWA icons

Add to `static/img/`:
- `icons/icon-72x72.png`
- `icons/icon-96x96.png`
- `icons/icon-128x128.png`
- `icons/icon-192x192.png`
- `icons/icon-512x512.png`

### Step 3: Configure in docusaurus.config.ts

```typescript
plugins: [
  [
    '@docusaurus/plugin-pwa',
    {
      offlineModeActivationStrategies: [
        'appInstalled',
        'standalone',
        'queryString',
      ],
      pwaHead: [
        {
          tagName: 'link',
          rel: 'icon',
          href: '/img/icons/icon-192x192.png',
        },
        {
          tagName: 'link',
          rel: 'manifest',
          href: '/manifest.json',
        },
        {
          tagName: 'meta',
          name: 'theme-color',
          content: '#00d4ff',
        },
      ],
    },
  ],
],
```

---

## TASK-006: Implement Local Search

### Step 1: Install search plugin

```bash
npm install @easyops-cn/docusaurus-search-local
```

### Step 2: Configure in docusaurus.config.ts

```typescript
themes: [
  [
    '@easyops-cn/docusaurus-search-local',
    {
      hashed: true,
      language: ['en'],
      highlightSearchTermsOnTargetPage: true,
      explicitSearchResultPath: true,
    },
  ],
],
```

### Step 3: Build and test

```bash
npm run build
npm run serve
```

Search only works in production build.

---

## 🔄 Execution Order

1. **TASK-001** - Code examples (foundation for downloads)
2. **TASK-002** - GitHub Actions (enables deployment)
3. **TASK-003** - Solutions (completes educational content)
4. **TASK-004** - Homepage (visual polish)
5. **TASK-005** - PWA (offline capability)
6. **TASK-006** - Search (discoverability)

---

## ✅ Verification Checklist

After each task:

```bash
# Verify build succeeds
npm run build

# Test locally
npm run serve
```

- [ ] No build errors
- [ ] Feature works as expected
- [ ] Mobile responsive
- [ ] Dark mode compatible

---

*Execute tasks in order. Update /sp.tasks after completing each.*
