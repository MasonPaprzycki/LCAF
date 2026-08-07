"""A 3D, neural-network open-die-forging displacement surrogate.

Implements the technique in Jagtap, Reinisch & Bailly, "Fast prediction of
the material displacement in open die forging using neural networks"
(ESAFORM 2024, DOI 10.21741/9781644903131-253), generalised from the
paper's 2D core-fibre cross-section to full 3D. See ``README.md`` in this
directory for the citation and ``docs/surrogate_deformation_model.md`` (in
the repository root ``docs/``) for the full method writeup.

Public entry points:

- ``inference.SurrogateNetwork`` -- load a trained checkpoint and apply it
  as a toolpath preview's deformation kernel
  (``lcaf.toolpathing.visualization.material_state`` calls this).
- ``train.main`` -- train a new checkpoint from JAX-FEM-generated data (or
  ``--dummy`` synthetic data for a smoke test); also runnable as
  ``python -m lcaf.simulation.surrogate.train``.
- ``checkpoint.load``/``checkpoint.save`` -- the ``.npz`` checkpoint format.
"""

from .checkpoint import Checkpoint, load as load_checkpoint, save as save_checkpoint
from .inference import SurrogateDomainWarning, SurrogateNetwork
from .process_params import ProcessParameters

__all__ = [
    "Checkpoint",
    "load_checkpoint",
    "save_checkpoint",
    "SurrogateDomainWarning",
    "SurrogateNetwork",
    "ProcessParameters",
]
