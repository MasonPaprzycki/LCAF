# toolpaths/

Generated ForgeBrain JSONL files go here. `python3 -m lcaf.toolpathing.ui`'s
**Export JSONL** button defaults to saving into this folder, and
`python3 -m lcaf.control.main` lists every `.jsonl` file here for you to
pick from at startup.

See [docs/toolpath_slicer_ui_guide.md](../docs/toolpath_slicer_ui_guide.md)
for how to generate one, and
[docs/software_setup.md](../docs/software_setup.md) for how to run one.

This folder is otherwise just a convenience default -- `lcaf.control.main`
also accepts a path to a `.jsonl` file anywhere else
(`python3 -m lcaf.control.main path/to/file.jsonl`).
