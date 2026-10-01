# FPV 2026 — IEEE report

IEEEtran two-column conference paper on the rotor-rate flight controller.

```
main.tex          the paper
refs.bib          bibliography
figures/*.pdf     eight vector figures (generated, do not hand-edit)
make_figures.py   regenerates figures/ from source constants and flight logs
preview/*.png     raster previews, for eyeballing only — not used by LaTeX
```

| Figure | Content |
|---|---|
| `fig_wiring` | System interconnect: power, ESC, buses, pin assignments |
| `fig_body` | Body frame, rotor layout, control-effectiveness matrix |
| `fig_estimator` | Which measurement observes which ESKF error state |
| `fig_cascade` | The six-loop control cascade |
| `fig_alloc` | Rotor-speed budget and saturation priority |
| `fig_states` | Flight state machine and hold engagement |
| `fig_hover` | Results: altitude, thrust and rotor speed traces |
| `fig_results` | Results: attitude, torque utilisation, XY error |

## Compiling

There is no LaTeX toolchain on this machine, so compile on Overleaf.

1. Go to overleaf.com → **New Project** → **Upload Project**.
2. Zip and upload `main.tex`, `refs.bib` and the whole `figures/` folder.
   `preview/` and `make_figures.py` are not needed and can be left out.
3. Overleaf ships IEEEtran, so it compiles as-is. Set the compiler to
   **pdfLaTeX** if it is not already.
4. Compile twice (Overleaf does this automatically) so the bibliography and
   cross-references resolve.

If you would rather build locally, install TeX Live or MiKTeX and run:

```bash
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

## Sections still to write

Search the source for `\stub` — there are six, each rendered as a grey
outlined box in the PDF so they are impossible to miss on a read-through:

| Location | What is missing |
|---|---|
| §II Transmitter | Photographs of the transmitter, airframe, and sensor placement relative to the power leads |
| §III Rigid body | Bifilar-pendulum measurement of the inertia tensor |
| §X Method | Instrumented rate-loop step responses (bandwidth, phase margin) |
| §X Position hold | Why hold quality varies between flights |
| §XI Identification | A valid k<sub>M</sub> run under uncorrelated yaw excitation |
| Acknowledgment | Advisor, course, lab support |

Plus `refs.bib`: four of your Reference Material PDFs are commented out. I
could not read their titles or authors from the files, so I left them blank
rather than guessing at citations.

Delete the `\stub{...}` call once you have written the real text. To remove
the boxes entirely without writing the content, redefine the macro near the
top of `main.tex`:

```latex
\renewcommand{\stub}[1]{}
```

## Page count

The paper is written to land at roughly ten pages. Four of the seven figures
are full-width (`figure*`), which is where most of the page area goes. If it
overruns:

- `fig_wiring`, `fig_body` and `fig_cascade` can each drop from `figure*` to a
  single-column `figure` by regenerating them at `COL1` width, at the cost of
  legibility;
- §XI (in-flight parameter identification) is the most self-contained section
  and can be cut to a paragraph in §XII without breaking the argument.

## Regenerating figures

```bash
python make_figures.py            # writes figures/*.pdf
python make_figures.py --preview  # also writes preview/*.png
```

Requires `matplotlib` and `numpy`, both already installed. Fonts are embedded
as Type 42; IEEE rejects Type 3, so do not change `pdf.fonttype`.

The two results figures read `logs/hover05_20260815_152920.log` directly, via
the `FLIGHT` constant near the bottom of the script. Point it at a newer log
and the figures and every number in §X will need to be re-derived — the
statistics quoted in the text are **not** generated automatically.

## Where the numbers come from

Every constant in Table I is derived from `FrameConventions.h` in the receiver
firmware, not typed in by hand from notes. If you change `PROP_THRUST_CT`,
`AIRFRAME_MASS_KG` or the frame separations, Table I and the authority figures
in §III go stale.

Flight statistics in §X come from the 15 August hover log:

- 5694 telemetry frames, 917 airborne, 898 in hover
- longest contiguous airborne segment: 348 frames, 69 s
- steady-hover subset: 603 frames with |VZ| ≤ 0.05 m/s and altitude hold engaged
