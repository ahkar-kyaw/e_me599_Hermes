# Thesis figure sources

Replace each in-text `<INSERT FIGURE ...>` block with a publication-quality figure when its source becomes available. Store editable source files in a clearly named subdirectory and export the version used by LaTeX as PDF for vector art or PNG for raster photographs.

Recommended insertion pattern:

```tex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=0.9\linewidth]{figures/ch04/zephyr_as_built.pdf}
  \caption{Current Zephyr mechanical prototype with major components identified.}
  \label{fig:zephyr-as-built}
\end{figure}
```

The `equation_render_reference` directory contains the equation images recovered from the Word draft for visual comparison only. The thesis chapters use editable LaTeX equations instead.
