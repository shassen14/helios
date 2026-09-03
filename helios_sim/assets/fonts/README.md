# Fonts

The inspector panel (`viz/interaction/inspector/render.rs`) loads its text face
from here. Bevy's default embedded font only covers ASCII, so glyphs like `°`
(degree) and `·` (the middle dot in `N·m`) render as the missing-glyph box.

Drop a redistributable TTF with Latin-1 Supplement coverage at:

    fonts/DejaVuSans.ttf

matching `UI_FONT_PATH` in `render.rs`. DejaVu Sans, Noto Sans, and Inter all
work and are freely redistributable. If the file is absent the panel still
renders — the renderer falls back to the default face (with the box glyphs).

To use a different filename, change `UI_FONT_PATH`.
