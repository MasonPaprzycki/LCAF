# Runnable Geometry Examples

All example targets are watertight OBJ meshes in millimetres and use X as the
billet axis. They are intentionally small enough for the default machine travel
limits and a 10 mm-radius cylindrical stock.

| Target | Shape | Suggested stock radius | Suggested axial spacing |
| --- | --- | ---: | ---: |
| `square_bar.obj` | 20 mm long, 10 mm square bar | 10 mm | 10 mm |
| `hex_bar.obj` | 30 mm long, 6-sided bar | 10 mm | 5 mm |
| `tapered_square_bar.obj` | 60 mm square taper, 8 → 16 → 8 mm | 10 mm | 5 mm |
| `tapered_hex_bar.obj` | 60 mm hex taper, radius 5 → 8 → 5 mm | 10 mm | 5 mm |

Use **Example target** in the slicer UI to load one with one click, then
generate its preview. The four-face examples use A = 0/90/180/270 degrees; the
current checked-in machine configuration is only A = -90 to +90 degrees. They
are therefore suitable for visualisation by default only after explicitly
selecting the continuous/indexed rotary override. They are not ready-to-run
machine programs on that configuration.

For a CLI visualisation/export demonstration with proven continuous/indexed A
hardware, for example:

```powershell
$env:PYTHONPATH = (Resolve-Path .\src)
python -m lcaf.toolpathing .\examples\tapered_hex_bar.obj .\output\tapered_hex_bar.jsonl --stock-radius 10 --axial-resolution 5 --rotation-step 90 --max-reduction 2 --die-contact-z 10 --allow-out-of-limit-rotations
```
