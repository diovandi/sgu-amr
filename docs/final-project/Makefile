# Makefile for LaTeX Final Project Report
# Domestic Mobile Robot - Nav2 & SLAM

# Main document name (without .tex extension)
MAIN = final_project_report

# LaTeX compiler
LATEX = pdflatex
BIBTEX = bibtex
MAKEINDEX = makeindex

# Compilation flags
LATEX_FLAGS = -interaction=nonstopmode -halt-on-error

# Output directory
OUTPUT_DIR = compiled

# Auxiliary files to clean
AUX_FILES = *.aux *.log *.toc *.lof *.lot *.out *.bbl *.blg *.nlo *.nls *.ilg *.idx *.ind

.PHONY: all clean cleanall view help

# Default target
all: $(MAIN).pdf

# Full compilation with bibliography and nomenclature
$(MAIN).pdf: $(MAIN).tex references.bib preamble.tex commands.tex
	@echo "=== First LaTeX pass ==="
	$(LATEX) $(LATEX_FLAGS) $(MAIN).tex
	@echo "\n=== Running BibTeX ==="
	-$(BIBTEX) $(MAIN)
	@echo "\n=== Running makeindex for nomenclature ==="
	-$(MAKEINDEX) $(MAIN).nlo -s nomencl.ist -o $(MAIN).nls
	@echo "\n=== Second LaTeX pass ==="
	$(LATEX) $(LATEX_FLAGS) $(MAIN).tex
	@echo "\n=== Third LaTeX pass ==="
	$(LATEX) $(LATEX_FLAGS) $(MAIN).tex
	@echo "\n=== Moving PDF to output directory ==="
	@mkdir -p $(OUTPUT_DIR)
	@mv $(MAIN).pdf $(OUTPUT_DIR)/
	@echo "\n✓ Compilation complete: $(OUTPUT_DIR)/$(MAIN).pdf"

# Quick compilation (no bibliography update)
quick: $(MAIN).tex
	@echo "=== Quick LaTeX compilation ==="
	$(LATEX) $(LATEX_FLAGS) $(MAIN).tex
	@mkdir -p $(OUTPUT_DIR)
	@mv $(MAIN).pdf $(OUTPUT_DIR)/
	@echo "\n✓ Quick compilation complete: $(OUTPUT_DIR)/$(MAIN).pdf"

# Clean auxiliary files
clean:
	@echo "=== Cleaning auxiliary files ==="
	@rm -f $(AUX_FILES)
	@echo "✓ Auxiliary files cleaned"

# Clean everything including PDF
cleanall: clean
	@echo "=== Cleaning all generated files ==="
	@rm -f $(OUTPUT_DIR)/$(MAIN).pdf
	@rm -rf $(OUTPUT_DIR)
	@echo "✓ All generated files cleaned"

# View the PDF (Linux)
view: $(OUTPUT_DIR)/$(MAIN).pdf
	@if command -v xdg-open > /dev/null; then \
		xdg-open $(OUTPUT_DIR)/$(MAIN).pdf & \
	elif command -v evince > /dev/null; then \
		evince $(OUTPUT_DIR)/$(MAIN).pdf & \
	else \
		echo "No PDF viewer found. Please open $(OUTPUT_DIR)/$(MAIN).pdf manually."; \
	fi

# Help message
help:
	@echo "LaTeX Final Project Report - Makefile Help"
	@echo "=========================================="
	@echo ""
	@echo "Available targets:"
	@echo "  make          - Full compilation with bibliography and nomenclature"
	@echo "  make all      - Same as 'make'"
	@echo "  make quick    - Quick compilation without bibliography update"
	@echo "  make clean    - Remove auxiliary files"
	@echo "  make cleanall - Remove all generated files including PDF"
	@echo "  make view     - Open the PDF in default viewer"
	@echo "  make help     - Show this help message"
	@echo ""
	@echo "Output location: $(OUTPUT_DIR)/$(MAIN).pdf"

