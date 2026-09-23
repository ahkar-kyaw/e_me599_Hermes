# Zephyr Master's Thesis — LaTeX Workshop Project

This project is the LaTeX conversion of the complete first-draft thesis:

**Design and Development of a Self-Balancing Mobile Robot with a Collinear Four-Wheel Mecanum Drive**

The project is configured for Visual Studio Code with the **LaTeX Workshop** extension. It uses XeLaTeX for Unicode and Times-compatible fonts, BibTeX with IEEE formatting, `latexmk` for dependency management, and a `build/` output directory.

## Quick start in VS Code

1. Install a full TeX distribution:
   - macOS: MacTeX
   - Windows: TeX Live or MiKTeX
   - Linux: TeX Live with XeLaTeX, `latexmk`, BibTeX, and the standard science/publishing packages
2. Install the VS Code extension **LaTeX Workshop** by James Yu.
3. Open this directory as the VS Code folder.
4. Open `main.tex`.
5. Save the file or run **LaTeX Workshop: Build LaTeX project**.
6. Select the `latexmk (XeLaTeX + BibTeX)` recipe if prompted.

The compiled thesis will be written to `build/main.pdf`.

## Command-line build

```bash
latexmk -xelatex main.tex
```

Clean generated files with:

```bash
latexmk -C main.tex
```

## Project structure

- `main.tex` — root document and university-format configuration
- `config/metadata.tex` — author, degree, committee, and submission fields
- `frontmatter/` — title, copyright, committee, abstract, acknowledgments, symbols, and abbreviations
- `chapters/` — Chapters 1–12
- `appendices/` — Appendices A–M
- `references.bib` — verified IEEE-style reference database
- `figures/` — figure assets and insertion guidance
- `.vscode/settings.json` — LaTeX Workshop build recipe
- `.latexmkrc` — reproducible XeLaTeX/BibTeX build configuration
- `preview/Zephyr_Thesis_Preview.pdf` — compiled review copy of the project

## Important editing notes

- Replace the author-information fields in `config/metadata.tex` first.
- Search the project for `AUTHOR INPUT REQUIRED`, `VERIFY FROM HARDWARE`, `INSERT FIGURE`, `INSERT DATA`, and `INSERT RESULT` to locate incomplete evidence.
- The four-wheel collinear Mecanum geometry remains symbolic until wheel coordinates and roller signs are measured. Do not substitute the conventional rectangular Mecanum matrix.
- Figure and table placeholders retain the proposed numbering from the Word draft. When replacing a placeholder with a native `figure` or `table` environment, add a `\label{...}` and update nearby references to use `\ref{...}`.
- Editable equations have replaced the equation images from the Word draft.
- Tables with five or more columns are set on landscape pages for legibility.
- Times New Roman is selected when it is installed; otherwise the project falls back to Latin Modern Roman.
- Rebuild twice after structural changes so the table of contents and lists settle; `latexmk` handles this automatically.

## Submission caution

This remains a first draft. The quantitative results, final hardware parameters, experimental plots, committee information, and several as-built verification items are intentionally marked as placeholders. Update the abstract and conclusions only after the referenced data have been collected and reviewed.
