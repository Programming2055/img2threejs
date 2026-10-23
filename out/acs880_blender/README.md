# ACS880 disassemble / assemble animation (Blender)

Files
- `acs880_explode_animation.py` – builds the whole animation in Blender: groups the shells, keyframes the
  explode and re-assemble with staggered, eased motion, adds materials, lights and an orbiting camera.
- `step_to_obj.py` – converts a STEP file to a per-shell OBJ (needs `pip install cadquery-ocp numpy`).
  Blender cannot import STEP natively. Usage: `python step_to_obj.py in.stp out.obj`
- `*.obj` – ready-made conversions (metres, Z up): ACS880-01 R4/R5/R6 P940 and ACS880-11/-31 R3/R6/R8 IP20.

Run (headless)
    blender --background --python acs880_explode_animation.py -- --obj ACS880-01_R4_P940.obj --out ./render --video explode.mp4

Run in an open Blender / Blender MCP: set `CONFIG["obj_path"]` (or import the model yourself and leave it empty),
paste the script into `execute_blender_code` or the Scripting tab and run. The timeline is 277 frames at 24 fps.
Tweak `explode_scale`, `stagger`, `orbit_degrees`, hold lengths in the CONFIG block.

Limits
- The ABB STEP files are single-product "shrinkwrap" surface models with anonymous shells and no colours, so the
  parts are auto-grouped by size/proximity. They are plausible plates/covers/boards, not ABB's real component names,
  and the colours are decorative.
- Verified headlessly with Blender 5.2 (`bpy`) on stills; a real Blender MCP session was not available here.
