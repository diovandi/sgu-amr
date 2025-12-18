# Final Project Report - Compilation Guide

## Domestic Mobile Robot: Navigation with Nav2 & SLAM

**Author:** Diovandi Basheera Putra  
**Course:** Autonomous Mobile Robot (2025-2026/2025-1-2943)  
**Instructor:** Dr. Rusman Rusyadi  
**Institution:** Swiss German University

---

## Overview

This document provides instructions for compiling the LaTeX-based final project report.

## Prerequisites

### Required Software

- **TeX Distribution:**
  - **Linux:** TeX Live (recommended: `texlive-full`)
  - **Windows:** MiKTeX
  - **macOS:** MacTeX

- **Build Tools:**
  - `pdflatex`
  - `bibtex` or `biber`
  - `makeindex`
  - `make` (optional, for using Makefile)

### Installation on Pop!_OS / Ubuntu

```bash
# Full TeX Live installation (recommended)
sudo apt-get update
sudo apt-get install texlive-full

# Minimal installation (if disk space is limited)
sudo apt-get install texlive-latex-extra texlive-science texlive-bibtex-extra \
                     texlive-fonts-extra texlive-publishers biber
```

## Compilation Methods

### Method 1: Using the Shell Script (Recommended)

The simplest method:

```bash
cd /home/dio/ros2_ws/docs
./compile_report.sh
```

This script will:
1. Run pdflatex (first pass)
2. Process bibliography with bibtex
3. Generate nomenclature with makeindex
4. Run pdflatex (second pass)
5. Run pdflatex (third pass for cross-references)
6. Move the PDF to `compiled/` directory

**Output:** `compiled/final_project_report.pdf`

### Method 2: Using Make

If you prefer using Make:

```bash
cd /home/dio/ros2_ws/docs
make
```

**Available Make targets:**
- `make` or `make all` - Full compilation
- `make quick` - Quick compilation (no bibliography update)
- `make clean` - Remove auxiliary files
- `make cleanall` - Remove all generated files
- `make view` - Open PDF in default viewer
- `make help` - Show help message

### Method 3: Manual Compilation

For manual control or debugging:

```bash
cd /home/dio/ros2_ws/docs

# First pass
pdflatex -interaction=nonstopmode final_project_report.tex

# Process bibliography
bibtex final_project_report

# Generate nomenclature
makeindex final_project_report.nlo -s nomencl.ist -o final_project_report.nls

# Second pass (resolve citations)
pdflatex -interaction=nonstopmode final_project_report.tex

# Third pass (resolve cross-references)
pdflatex -interaction=nonstopmode final_project_report.tex

# Move output
mkdir -p compiled
mv final_project_report.pdf compiled/
```

## File Structure

```
docs/
├── final_project_report.tex    # Main document
├── preamble.tex                # LaTeX preamble with packages
├── commands.tex                # Custom commands and macros
├── references.bib              # Bibliography database
├── Makefile                    # Make build system
├── compile_report.sh           # Shell compilation script
├── README_REPORT.md            # This file
├── screenshots/                # Figures and images (local only, in .gitignore)
│   ├── setup/
│   ├── lidar/
│   ├── nav/
│   └── patrol/
└── compiled/                   # Output directory
    └── final_project_report.pdf
```

## Screenshots

**Important:** Screenshots are stored locally in `docs/screenshots/` but are **not tracked by git** (in `.gitignore`). This keeps the repository size manageable while allowing the LaTeX document to reference local images during compilation.

### Screenshot Organization

- `screenshots/setup/` - Robot design and simulation setup images
- `screenshots/lidar/` - LIDAR processing and troubleshooting images
- `screenshots/nav/` - Navigation stack and Nav2 images
- `screenshots/patrol/` - Autonomous patrol and analysis images

## Troubleshooting

### Error: "pdflatex: command not found"

**Solution:** Install TeX Live as shown in Prerequisites section.

### Error: "File `*.sty' not found"

**Solution:** Install the missing package or use `texlive-full` for complete installation.

```bash
sudo apt-get install texlive-full
```

### Error: Bibliography not showing up

**Cause:** BibTeX may have failed or citations not present in text.

**Solution:**
1. Ensure you have `\cite{...}` commands in the document
2. Check `final_project_report.blg` for BibTeX errors
3. Recompile with the full sequence (3 pdflatex + 1 bibtex passes)

### Error: Cross-references showing "??"

**Cause:** Need additional compilation passes.

**Solution:** Run pdflatex one more time, or use the provided scripts which automatically do 3 passes.

### Compilation is very slow

**Cause:** Large number of figures or complex TikZ diagrams.

**Solution:**
- Use `make quick` for faster recompilation when only changing text
- Comment out heavy graphics during draft editing
- Use draft mode: `\documentclass[draft]{report}` in preamble

### Warning: "Overfull \hbox"

**Cause:** LaTeX couldn't break a line properly (often with URLs or code).

**Solution:** Usually cosmetic. Can be ignored unless severe. To fix:
- Break long URLs with `\url{...}` command
- Adjust code listing widths
- Use `\sloppy` paragraph mode for problematic sections

## Viewing the PDF

### Linux

```bash
# Using default PDF viewer
xdg-open compiled/final_project_report.pdf

# Or using specific viewers
evince compiled/final_project_report.pdf
okular compiled/final_project_report.pdf
```

### Windows

```bash
start compiled\final_project_report.pdf
```

### macOS

```bash
open compiled/final_project_report.pdf
```

## Alternative: Using Overleaf

If you prefer online LaTeX editing:

1. Create a new project on [Overleaf](https://www.overleaf.com)
2. Upload all files from `docs/` directory
3. Set `final_project_report.tex` as the main document
4. Compile online

**Note:** You'll need to manually upload screenshots to Overleaf.

## Document Structure

The report is organized as follows:

- **Front Matter:** Title, abstract, table of contents, lists, nomenclature
- **Chapter 1:** Introduction
- **Chapter 2:** Theoretical Foundations (EKF, SLAM, AMCL, Nav2)
- **Chapter 3:** Robot Design and Simulation
- **Chapter 4:** System Architecture
- **Chapter 5:** LIDAR Processing and Obstacle Detection
- **Chapter 6:** Mapping with SLAM Toolbox
- **Chapter 7:** Navigation Stack Implementation
- **Chapter 8:** Autonomous Patrol System
- **Chapter 9:** Performance Analysis
- **Chapter 10:** Integration and System Validation
- **Chapter 11:** Lessons Learned and Best Practices
- **Chapter 12:** Conclusions and Future Work
- **Appendices:** Configuration files, launch files, URDF specs, commands
- **References:** Bibliography

## Contact

For questions about the report compilation or content:

**Author:** Diovandi Basheera Putra  
**Institution:** Swiss German University  
**Course:** Autonomous Mobile Robot (2025-2026/2025-1-2943)

---

*Last updated: December 18, 2025*

