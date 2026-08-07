# ============================================================
# JAX-FEM TRAINING DATA GENERATOR FOR lcaf.simulation.surrogate
# ============================================================
#
# Generates the FEA ground-truth data
# lcaf.simulation.surrogate.train.py trains the 3D JAGTAP-et-al.-style
# displacement surrogate on (see docs/surrogate_deformation_model.md and
# docs/surrogate_training_guide.md).
#
# WHY THIS SCRIPT, ADAPTED FROM Hexagon.py:
#   Of the scripts in this folder, Hexagon.py is the closest match to this
#   machine's own geometry -- a rotating, multi-pass, J2-elastoplastic
#   open-die forging simulation on a rectangular-ish billet, hot steel
#   material, incremental load stepping. This script reuses its material
#   model (J2 radial-return plasticity with linear isotropic hardening) and
#   incremental solver-loop pattern nearly verbatim, but replaces the fixed
#   6-pass hexagon-forming schedule with a *single, generic, parametrised
#   strike* -- exactly the unit of data Jagtap, Reinisch & Bailly's own
#   network is trained on (one stroke, described by process parameters
#   alpha0/xb/eps_h + a reference-configuration point -> displacement).
#   Hexagon.py itself is left completely untouched.
#
# WHAT ONE SAMPLE IS:
#   A rectangular billet of height h0 (REFERENCE_HEIGHT_MM, matching the
#   paper's own 100 mm convention) and width w0 = h0/alpha0
#   (lcaf.simulation.surrogate.process_params.billet_dimensions_mm), long
#   enough (see BILLET_LENGTH_REACH_MULTIPLE) that the die's own zone of
#   influence never reaches the axial ends. A flat rigid anvil supports the
#   entire bottom face (Z=0); a flat rigid die presses the top face
#   (Z=h0) down by `reduction_mm(eps_h, h0)` over an axial window of width
#   `bite_length_mm(xb, h0)` centred on the billet -- both dies are
#   implicitly full-width across the billet's own spread (Y) direction,
#   matching the paper's own saddle assumption (see
#   lcaf/simulation/surrogate/README.md's scope section). This is the
#   "locally flat slab" every one of this machine's real strikes is treated
#   as, regardless of the billet's true (possibly non-rectangular, already
#   partly forged) cross-section -- see docs/surrogate_deformation_model.md.
#
# COORDINATE CONVENTION (matches lcaf.simulation.surrogate.geometry exactly):
#   FEM X (axial)          -> paper/network z0, re-based to 0 at the die's
#                              own leading (near) edge, i.e.
#                              z0 = X - (die_centre_x - bite/2).
#   FEM Y (spread/width)   -> paper/network x0, already 0 at the billet's
#                              own centreline by construction.
#   FEM Z (press/height)   -> paper/network y0, already 0 at the anvil
#                              (bottom face) by construction.
#   Displacement components map the same way: (u_x, u_y, u_z) -> (dz0, dx0, dy0).
#   This is why the die always presses in canonical "+Z, rotation=0": every
#   real strike's own rotation is handled entirely by
#   lcaf.simulation.surrogate.geometry.LocalFrame at *inference* time, not
#   here -- exactly the paper's own "repositioning" trick (train once,
#   reuse for any stroke by aligning the reference configuration first).
#
# MINIMAL RIGID-BODY CONSTRAINTS (do not distort the physics):
#   - Both axial end faces (X=0, X=L0): u_x = 0 (matches Hexagon.py's own
#     left_end/right_end trick).
#   - The bottom-centre ridge line (Y=0, Z=0, every X): u_y = 0. The whole
#     problem (geometry, every other BC) is already symmetric about Y=0, so
#     global rigid-body translation in Y is the only otherwise-undetermined
#     degree of freedom -- pinning a single line, not a whole face, does not
#     suppress real spread anywhere else the way constraining the full Y=0
#     midplane would.
#
# OUTPUT: one ``.npz`` per sample in ``--out``, with keys
#   alpha0, xb, eps_h              (scalars, this sample's process params)
#   x0, y0, z0, dx0, dy0, dz0      (n_nodes,) each
# -- exactly the schema lcaf.simulation.surrogate.dataset.load_sample expects.
#
# STATUS: written by close analogy to Hexagon.py's own working JAX-FEM
# call patterns; JAX-FEM itself only runs in the WSL/conda ``jax-fem-env``
# described in SetupJaxFEM.ipynb/InstallProcess.ipynb, not in this
# repository's default Windows sandbox, so this script has not been
# executed end-to-end. Run a small ``--samples 1 --nx 12 --ny 8 --nz 8``
# smoke test first (see docs/surrogate_training_guide.md) before a full
# generation run.
#
# ENVIRONMENT:
#   conda activate jax-fem-env
#   python generate_surrogate_training_data.py --samples 200 --seed 0 --out data/train
#   python generate_surrogate_training_data.py --samples 54  --seed 1 --out data/test
# ============================================================

import argparse
import math
import os
import sys
import time

import numpy as np
import jax
import jax.numpy as jnp

from jax_fem.problem import Problem
from jax_fem.solver import solver
from jax_fem.generate_mesh import box_mesh_gmsh, get_meshio_cell_type, Mesh

# lcaf.simulation.surrogate.process_params has no JAX/mesh dependency (see
# its own module docstring), so it is safe to import here even though the
# rest of lcaf.simulation.surrogate (model.py etc.) depends on a plain
# `jax` install this environment may not have wired up identically -- only
# process_params is actually needed for this script's geometry conversions.
_REPO_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
if _REPO_SRC not in sys.path:
    sys.path.insert(0, _REPO_SRC)

from lcaf.simulation.surrogate.process_params import (  # noqa: E402
    REFERENCE_HEIGHT_MM,
    bite_length_mm,
    billet_dimensions_mm,
    reduction_mm,
    sample_process_parameters,
)

# ============================================================
# [1] DEFAULTS
# ============================================================

# How many bite-lengths (or a flat minimum, mm) of clear billet the die's
# own zone of influence needs on each side of it, along X, so the fixed
# axial end constraints never contaminate the strike's own local result --
# matches lcaf.simulation.surrogate.geometry.affected_station_indices' own
# default reach_multiple=4.0 exactly, so the FEA ground truth and the
# runtime "how far can this strike possibly reach" assumption agree.
BILLET_LENGTH_REACH_MULTIPLE = 4.0
BILLET_LENGTH_MIN_MM = 200.0

STEEL_E = 50.0e9      # Pa, hot steel modulus (matches Hexagon.py's own ~1150 C estimate)
STEEL_NU = 0.30
STEEL_YIELD = 15.0e6  # Pa, hot yield stress
STEEL_HARDENING = 100.0e6  # Pa, linear isotropic hardening modulus

ATOL = 1e-6


def _lame(E, nu):
    mu = E / (2.0 * (1.0 + nu))
    lam = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    return mu, lam


_MU, _LAM = _lame(STEEL_E, STEEL_NU)


# ============================================================
# [2] MATERIAL MODEL -- J2 elastoplasticity with hardening
#
# Identical structure to Hexagon.py's own ForgingPlasticity: an incremental
# radial-return, only duplicated here (not imported) so this script stays a
# single, self-contained file and Hexagon.py stays untouched.
# ============================================================

class ForgingPlasticity(Problem):
    def custom_init(self):
        self.fe = self.fes[0]
        nc = len(self.fe.cells)
        nq = self.fe.num_quads
        dim = self.dim

        self.sigmas_old = np.zeros((nc, nq, dim, dim))
        self.epsilons_old = np.zeros((nc, nq, dim, dim))
        self.eps_p_eq_old = np.zeros((nc, nq))

        self.internal_vars = [self.sigmas_old, self.epsilons_old, self.eps_p_eq_old]

    def get_tensor_map(self):
        _, stress_return_map = self._get_constitutive_maps()
        return stress_return_map

    def _get_constitutive_maps(self):
        mu, lam = _MU, _LAM
        sig0 = float(STEEL_YIELD)
        H = float(STEEL_HARDENING)
        dim = 3

        def safe_sqrt(x):
            return jnp.where(x > 0.0, jnp.sqrt(x), 0.0)

        def safe_div(x, y):
            return jnp.where(y == 0.0, 0.0, x / y)

        def strain_from_grad(u_grad):
            return 0.5 * (u_grad + u_grad.T)

        def elastic_stress(eps):
            return lam * jnp.trace(eps) * jnp.eye(dim) + 2.0 * mu * eps

        def stress_return_map(u_grad, sigma_old, epsilon_old, eps_p_eq_old):
            eps_crt = strain_from_grad(u_grad)
            delta_eps = eps_crt - epsilon_old
            sigma_tr = sigma_old + elastic_stress(delta_eps)

            s_dev = sigma_tr - (1.0 / dim) * jnp.trace(sigma_tr) * jnp.eye(dim)
            s_norm = safe_sqrt(1.5 * jnp.sum(s_dev * s_dev))

            sig_y = sig0 + H * eps_p_eq_old
            f_yield = s_norm - sig_y
            d_gamma = jnp.where(f_yield > 0.0, f_yield / (2.0 * mu + (2.0 / 3.0) * H), 0.0)

            n_hat = safe_div(s_dev, s_norm)
            return sigma_tr - 2.0 * mu * d_gamma * n_hat

        return strain_from_grad, stress_return_map

    def stress_strain_fns(self):
        strain_fn, srm = self._get_constitutive_maps()
        vmap_strain = jax.vmap(jax.vmap(strain_fn))
        vmap_srm = jax.vmap(jax.vmap(srm))
        return vmap_strain, vmap_srm

    def update_internal_vars(self, sol):
        u_grads = self.fe.sol_to_grad(sol)
        vmap_strain, vmap_srm = self.stress_strain_fns()

        new_sigma = vmap_srm(u_grads, self.sigmas_old, self.epsilons_old, self.eps_p_eq_old)
        new_epsilon = vmap_strain(u_grads)

        mu, H, sig0 = _MU, float(STEEL_HARDENING), float(STEEL_YIELD)
        delta_eps = new_epsilon - self.epsilons_old
        sigma_tr = self.sigmas_old + (
            _LAM * jnp.trace(delta_eps, axis1=-2, axis2=-1)[..., None, None] * jnp.eye(3)
            + 2.0 * mu * delta_eps
        )
        s_dev = sigma_tr - (1.0 / 3.0) * jnp.trace(sigma_tr, axis1=-2, axis2=-1)[..., None, None] * jnp.eye(3)
        s_norm = jnp.sqrt(jnp.maximum(1.5 * jnp.sum(s_dev * s_dev, axis=(-2, -1)), 0.0))
        sig_y = sig0 + H * self.eps_p_eq_old
        f_y = s_norm - sig_y
        d_gamma = jnp.where(f_y > 0.0, f_y / (2.0 * mu + (2.0 / 3.0) * H), 0.0)
        new_eps_p_eq = self.eps_p_eq_old + (
            jnp.sqrt(2.0 / 3.0) * 2.0 * mu / (2.0 * mu + (2.0 / 3.0) * H)
        ) * d_gamma

        self.sigmas_old = np.array(new_sigma)
        self.epsilons_old = np.array(new_epsilon)
        self.eps_p_eq_old = np.array(new_eps_p_eq)
        self.internal_vars = [self.sigmas_old, self.epsilons_old, self.eps_p_eq_old]


# ============================================================
# [3] ONE-SAMPLE MESH + BOUNDARY CONDITIONS
# ============================================================

def _build_mesh(length_m, width_m, height_m, nx, ny, nz, ele_type, data_dir):
    meshio_mesh = box_mesh_gmsh(
        Nx=nx, Ny=ny, Nz=nz,
        domain_x=length_m, domain_y=width_m, domain_z=height_m,
        data_dir=data_dir, ele_type=ele_type,
    )
    cell_type = get_meshio_cell_type(ele_type)
    mesh = Mesh(np.array(meshio_mesh.points), meshio_mesh.cells_dict[cell_type])
    mesh.points = np.array(mesh.points)
    # Centre Y (spread) at 0; Z (press) already starts at 0 = the anvil.
    mesh.points[:, 1] -= width_m / 2.0
    return mesh


def _make_bc_info(mesh, length_m, height_m, die_x0_m, die_x1_m, frac):
    """Dirichlet BC info for one load increment (see the module docstring's
    "MINIMAL RIGID-BODY CONSTRAINTS" section for the X/Y pinning).
    """

    def bottom_z(point):
        return jnp.abs(point[2]) < ATOL

    def top_die_face(point):
        in_x = (point[0] >= die_x0_m - ATOL) & (point[0] <= die_x1_m + ATOL)
        on_top = jnp.abs(point[2] - height_m) < ATOL
        return in_x & on_top

    def left_end(point):
        return jnp.abs(point[0]) < ATOL

    def right_end(point):
        return jnp.abs(point[0] - length_m) < ATOL

    def bottom_centre_ridge(point):
        return (jnp.abs(point[2]) < ATOL) & (jnp.abs(point[1]) < ATOL)

    def zero_disp(point):
        return 0.0

    def die_disp_z(point, value=-frac):
        return value

    location_fns = [bottom_z, top_die_face, left_end, right_end, bottom_centre_ridge]
    vecs = [2, 2, 0, 0, 1]
    value_fns = [zero_disp, die_disp_z, zero_disp, zero_disp, zero_disp]
    return [location_fns, vecs, value_fns]


def _run_one_sample(alpha0, xb, eps_h, nx, ny, nz, n_increments, ele_type, data_dir, verbose=True):
    """Press one billet by one strike; return reference/displaced node clouds
    in the local (x0, y0, z0)/(dx0, dy0, dz0) convention (see module docstring).
    """
    h0_mm, w0_mm = billet_dimensions_mm(alpha0, REFERENCE_HEIGHT_MM)
    bite_mm = bite_length_mm(xb, h0_mm)
    reduction_total_mm = reduction_mm(eps_h, h0_mm)
    length_mm = max(BILLET_LENGTH_REACH_MULTIPLE * bite_mm * 2.0, BILLET_LENGTH_MIN_MM)

    # JAX-FEM/petsc numerics are conditioned around O(1) geometry; work in
    # metres internally (mm / 1000), matching Hexagon.py's own inch->metre
    # convention of keeping the FEM solve in SI units.
    length_m, width_m, height_m = length_mm / 1000.0, w0_mm / 1000.0, h0_mm / 1000.0
    bite_m = bite_mm / 1000.0
    reduction_m = reduction_total_mm / 1000.0
    die_centre_m = length_m / 2.0
    die_x0_m, die_x1_m = die_centre_m - bite_m / 2.0, die_centre_m + bite_m / 2.0

    if verbose:
        print(
            f"    billet: L={length_mm:.1f}mm W={w0_mm:.1f}mm H={h0_mm:.1f}mm  "
            f"bite={bite_mm:.1f}mm  reduction={reduction_total_mm:.2f}mm"
        )

    mesh = _build_mesh(length_m, width_m, height_m, nx, ny, nz, ele_type, data_dir)
    reference_points_m = np.array(mesh.points, copy=True)

    initial_bc = _make_bc_info(mesh, length_m, height_m, die_x0_m, die_x1_m, frac=0.0)
    problem = ForgingPlasticity(mesh=mesh, vec=3, dim=3, ele_type=ele_type, dirichlet_bc_info=initial_bc)

    sol = None
    for increment in range(1, n_increments + 1):
        frac = reduction_m * increment / n_increments
        bc_info = _make_bc_info(mesh, length_m, height_m, die_x0_m, die_x1_m, frac=frac)
        problem.fe.update_Dirichlet_boundary_conditions(bc_info)
        sol_list = solver(problem, solver_options={"petsc_solver": {}})
        sol = sol_list[0]
        problem.update_internal_vars(sol)
        if verbose:
            print(f"      increment {increment}/{n_increments}  frac={frac * 1000.0:.3f}mm")

    displacement_m = np.array(sol).reshape(-1, 3)

    # FEM (X, Y, Z) -> network (z0, x0, y0); see the module docstring.
    x0_mm = reference_points_m[:, 1] * 1000.0
    y0_mm = reference_points_m[:, 2] * 1000.0
    z0_mm = reference_points_m[:, 0] * 1000.0 - (die_centre_m - bite_m / 2.0) * 1000.0
    dx0_mm = displacement_m[:, 1] * 1000.0
    dy0_mm = displacement_m[:, 2] * 1000.0
    dz0_mm = displacement_m[:, 0] * 1000.0

    return {
        "alpha0": np.array(alpha0), "xb": np.array(xb), "eps_h": np.array(eps_h),
        "x0": x0_mm, "y0": y0_mm, "z0": z0_mm,
        "dx0": dx0_mm, "dy0": dy0_mm, "dz0": dz0_mm,
    }


# ============================================================
# [4] MAIN
# ============================================================

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=200, help="Matches the paper's own 200-sample training set by default.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=str, default="surrogate_training_data")
    parser.add_argument("--nx", type=int, default=40, help="Elements along the billet's axial (X) length.")
    parser.add_argument("--ny", type=int, default=16, help="Elements across the spread (Y) direction.")
    parser.add_argument("--nz", type=int, default=16, help="Elements across the press (Z) direction.")
    parser.add_argument("--n-increments", type=int, default=8, help="Load steps ramping the die from 0 to its full reduction.")
    parser.add_argument("--start-index", type=int, default=0, help="First sample's file index (for resuming/parallel runs).")
    args = parser.parse_args(argv)

    print("=" * 65)
    print("  SURROGATE TRAINING DATA GENERATOR (JAX-FEM)")
    print("=" * 65)
    print(f"  JAX version: {jax.__version__}")
    print(f"  Devices: {jax.devices()}")

    os.makedirs(args.out, exist_ok=True)
    mesh_data_dir = os.path.join(args.out, "_mesh_cache")
    os.makedirs(mesh_data_dir, exist_ok=True)
    ele_type = "HEX8"

    process_parameters = sample_process_parameters(args.samples, args.seed)
    print(f"  Sampling {args.samples} strikes from the paper's own variable space (seed={args.seed}).")

    for offset, process in enumerate(process_parameters):
        index = args.start_index + offset
        out_path = os.path.join(args.out, f"sample_{index:04d}.npz")
        if os.path.exists(out_path):
            print(f"  [{offset + 1}/{args.samples}] {out_path} already exists, skipping.")
            continue

        print(f"  [{offset + 1}/{args.samples}] alpha0={process.alpha0:.3f} xb={process.xb:.3f} eps_h={process.eps_h:.3f}")
        started = time.time()
        sample = _run_one_sample(
            process.alpha0, process.xb, process.eps_h,
            args.nx, args.ny, args.nz, args.n_increments, ele_type, mesh_data_dir,
        )
        np.savez(out_path, **sample)
        print(f"    saved {out_path} ({sample['x0'].shape[0]} nodes) in {time.time() - started:.1f}s")

    print()
    print(f"  Done. {args.samples} samples written to {args.out}")
    print("  Next: lcaf.simulation.surrogate.train --data <this dir> --out <checkpoint.npz>")
    print("  (see docs/surrogate_training_guide.md)")


if __name__ == "__main__":
    main()
