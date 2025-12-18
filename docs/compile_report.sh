#!/bin/bash
# Compilation script for Final Project Report
# Domestic Mobile Robot - Nav2 & SLAM

set -e  # Exit on error

# Change to script directory
cd "$(dirname "$0")"

echo "======================================"
echo "LaTeX Final Project Report Compilation"
echo "======================================"
echo ""

# Check if pdflatex is installed
if ! command -v pdflatex &> /dev/null; then
    echo "Error: pdflatex not found. Please install TeX Live or MiKTeX."
    echo ""
    echo "On Ubuntu/Pop!_OS, install with:"
    echo "  sudo apt-get install texlive-full"
    exit 1
fi

# Check if bibtex is installed
if ! command -v bibtex &> /dev/null; then
    echo "Error: bibtex not found. Please install TeX Live or MiKTeX."
    exit 1
fi

# Main document name
MAIN="final_project_report"

echo "Step 1/5: First LaTeX pass..."
pdflatex -interaction=nonstopmode -halt-on-error "$MAIN.tex" || {
    echo "Error in first LaTeX pass. Check $MAIN.log for details."
    exit 1
}

echo ""
echo "Step 2/5: Running BibTeX..."
bibtex "$MAIN" || {
    echo "Warning: BibTeX reported errors. Check $MAIN.blg for details."
    echo "Continuing anyway..."
}

echo ""
echo "Step 3/5: Running makeindex for nomenclature..."
makeindex "$MAIN.nlo" -s nomencl.ist -o "$MAIN.nls" 2>/dev/null || {
    echo "Warning: makeindex failed. Nomenclature may not be generated."
    echo "Continuing anyway..."
}

echo ""
echo "Step 4/5: Second LaTeX pass..."
pdflatex -interaction=nonstopmode -halt-on-error "$MAIN.tex" || {
    echo "Error in second LaTeX pass. Check $MAIN.log for details."
    exit 1
}

echo ""
echo "Step 5/5: Third LaTeX pass (final)..."
pdflatex -interaction=nonstopmode -halt-on-error "$MAIN.tex" || {
    echo "Error in third LaTeX pass. Check $MAIN.log for details."
    exit 1
}

echo ""
echo "Moving PDF to compiled/ directory..."
mkdir -p compiled
mv "$MAIN.pdf" compiled/

echo ""
echo "======================================"
echo "✓ Compilation successful!"
echo "======================================"
echo ""
echo "Output: compiled/$MAIN.pdf"
echo ""
echo "To view the PDF:"
echo "  xdg-open compiled/$MAIN.pdf"
echo ""

# Optionally open the PDF
read -p "Open PDF now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if command -v xdg-open &> /dev/null; then
        xdg-open "compiled/$MAIN.pdf" &
    elif command -v evince &> /dev/null; then
        evince "compiled/$MAIN.pdf" &
    else
        echo "No PDF viewer found. Please open compiled/$MAIN.pdf manually."
    fi
fi

