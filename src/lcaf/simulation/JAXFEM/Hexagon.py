# ============================================================
# JAX-FEM HEXAGONAL ROD FORGING SIMULATION
# ============================================================
#
# PHYSICAL PROCESS:
#   A hot steel billet (24" × 4" × 4") sits on an anvil.
#   Phase A (6 passes): die presses region 16"–20" at 60°
#     rotational increments → produces large hex (4.0" AF).
#   Phase B (6 passes): die presses region 20"–24" deeper →
#     produces small hex (2.75" AF).
#
# KEY SIMPLIFICATIONS vs Genesis MPM:
#   - No rigid body contact. Die & anvil = Dirichlet BCs.
#   - No friction, thermal, recrystallisation, rate effects.
#   - J2 elastoplasticity with isotropic hardening.
#   - Large-deformation small-strain (incremental formulation).
#
# MATERIAL: Hot steel at ~1150°C
#   E ≈ 50 GPa (hot), nu = 0.30, yield ≈ 15 MPa, hardening.
#
# ANIMATION:
#   In-house matplotlib, matching Genesis script aesthetic:
#     - Dark background
#     - 3D convex hull coloured by equiv. plastic strain
#     - YZ cross-section with target hex overlay
#     - XZ side view with die / anvil boxes
#     - Diagnostics info strip
#   Output: output/forge_hex_jaxfem.mp4 + per-pass PNGs
#
# MODULES (easy to debug individually):
#   [1] PARAMETERS & GEOMETRY
#   [2] MESH GENERATION
#   [3] MATERIAL MODEL (J2 plasticity)
#   [4] BOUNDARY CONDITION HELPERS
#   [5] PROBLEM CLASS
#   [6] FORGING PASS RUNNER
#   [7] ANIMATION RENDERER
#   [8] MAIN LOOP
#
# ENVIRONMENT:
#   conda activate jax-fem-env
#   python forge_hex_jaxfem.py
# ============================================================

import os, sys, math, time, warnings
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import cv2
from scipy.spatial import ConvexHull

warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

# ── JAX / JAX-FEM imports ────────────────────────────────────
import jax
import jax.numpy as jnp

from jax_fem.problem import Problem
from jax_fem.solver import solver
from jax_fem.utils import save_sol
from jax_fem.generate_mesh import box_mesh_gmsh, get_meshio_cell_type, Mesh

print("="*65)
print("  JAX-FEM HEXAGONAL ROD FORGING SIMULATION")
print("="*65)
print(f"  JAX version: {jax.__version__}")
print(f"  Devices: {jax.devices()}")
print()

# ============================================================
# [1] PARAMETERS & GEOMETRY
# ============================================================

INCH = 0.0254   # metres per inch

# ── Billet ────────────────────────────────────────────────────
ROD_LEN = 24.0 * INCH   # 0.6096 m
ROD_Y   =  4.0 * INCH   # 0.1016 m
ROD_Z   =  4.0 * INCH   # 0.1016 m

# ── Tool geometry (for visualisation only) ───────────────────
ANVIL_THICKNESS = 0.75 * INCH
DIE_THICKNESS   = 0.75 * INCH
DIE_WIDTH_Y     = ROD_Y * 2.5

# ── Phase A: 16"–20" region, large hex 4.0" AF ───────────────
DIE_A_X0    = 16.0 * INCH
DIE_A_X1    = 20.0 * INCH
DIE_A_LEN   = DIE_A_X1 - DIE_A_X0          # 4.0"
DIE_A_CX    = (DIE_A_X0 + DIE_A_X1) / 2.0

HEX_F2F_A   = 3.5 * INCH                   # 4.0" flat-to-flat
HEX_APOTH_A = HEX_F2F_A / 2.0              # 50.8 mm

# ── Phase B: 20"–24" region, small hex 2.75" AF ──────────────
DIE_B_X0    = 20.0 * INCH
DIE_B_X1    = 24.0 * INCH
DIE_B_LEN   = DIE_B_X1 - DIE_B_X0
DIE_B_CX    = (DIE_B_X0 + DIE_B_X1) / 2.0

HEX_F2F_B   = 2.75 * INCH                  # 2.75" flat-to-flat
HEX_APOTH_B = HEX_F2F_B / 2.0             # 34.925 mm

# ── Hot steel 1150°C material ────────────────────────────────
# E drops dramatically at forging temp. 50 GPa is a reasonable
# estimate for austenitic steel near 1150°C. Yield ~15 MPa.
STEEL_E          = 50.0e9   # Pa
STEEL_NU         = 0.30
STEEL_RHO        = 7700.0   # kg/m³
STEEL_YIELD      = 15.0e6   # Pa (hot)
STEEL_HARDENING  = 100.0e6  # Pa (tangent modulus, light hardening)

# ── Forging passes ────────────────────────────────────────────
N_PASSES_A      = 6
N_PASSES_B      = 6
N_INCREMENTS    = 10   # load increments per press (more = smoother)

# ── Mesh resolution ───────────────────────────────────────────
# Coarse-ish for speed; refine by increasing these.
# FEM convergence is less resolution-hungry than MPM.
NX = 48   # elements along rod length (24" → 0.5" per element)
NY = 8    # elements across Y (4" → 0.5" per element)
NZ = 8    # elements across Z (4" → 0.5" per element)

# ── Output ────────────────────────────────────────────────────
OUT_DIR      = "output_jaxfem"
VTK_DIR      = os.path.join(OUT_DIR, "vtk")
FRAME_DIR    = os.path.join(OUT_DIR, "frames")
VIDEO_PATH   = os.path.join(OUT_DIR, "forge_hex_jaxfem.mp4")
ANIM_W, ANIM_H = 1440, 900
FPS          = 12   # lower FPS → each pass gets a few frames

for d in (OUT_DIR, VTK_DIR, FRAME_DIR):
    os.makedirs(d, exist_ok=True)

# ── Coordinate frame (Z=0 at anvil top / billet bottom) ───────
ANVIL_TOP_Z    = 0.0
billet_bottom  = 0.0
billet_top     = ROD_Z                      # 101.6 mm
billet_cx      = ROD_LEN / 2.0
billet_cy      = 0.0
billet_cz      = ROD_Z / 2.0
anvil_cz_vis   = -ANVIL_THICKNESS / 2.0    # mesh centre for viz

# ── Sanity ────────────────────────────────────────────────────
assert HEX_APOTH_A < ROD_Z / 2,  "Phase A: no compression needed"
assert HEX_APOTH_B < HEX_APOTH_A, "Phase B not deeper than A"
print(f"  Phase A compression: {(ROD_Z/2 - HEX_APOTH_A)*1e3:.2f} mm per face")
print(f"  Phase B compression: {(ROD_Z/2 - HEX_APOTH_B)*1e3:.2f} mm per face")
print(f"  Mesh: {NX}×{NY}×{NZ} = {NX*NY*NZ} elements")
print()

# ============================================================
# [2] MESH GENERATION
# ============================================================

print("  Generating mesh …")
ele_type  = "HEX8"
cell_type = get_meshio_cell_type(ele_type)

# box_mesh_gmsh expects keyword args: Nx, Ny, Nz, domain_x, domain_y, domain_z
# Billet sits with corner at (0,0,0) → (ROD_LEN, ROD_Y, ROD_Z).
# We offset Y so billet is centred at Y=0: Y ∈ [-ROD_Y/2, +ROD_Y/2].
# JAX-FEM box_mesh_gmsh always creates [0, Lx] × [0, Ly] × [0, Lz].
# We keep billet at [0,ROD_LEN] × [0,ROD_Y] × [0,ROD_Z] and offset
# in our BC helpers / viz (billet_cy_offset = ROD_Y/2).
BILLET_Y_OFFSET = ROD_Y / 2.0   # so physical Y centre = 0

meshio_mesh = box_mesh_gmsh(
    Nx=NX, Ny=NY, Nz=NZ,
    domain_x=ROD_LEN,
    domain_y=ROD_Y,
    domain_z=ROD_Z,
    data_dir=OUT_DIR,
    ele_type=ele_type,
)
mesh = Mesh(
    np.array(meshio_mesh.points),
    meshio_mesh.cells_dict[cell_type]
)

# Shift mesh so Y is centred at 0, Z bottom at 0
# mesh.points is (N_nodes, 3): x in [0,ROD_LEN], y in [0,ROD_Y], z in [0,ROD_Z]
mesh.points = np.array(mesh.points)
mesh.points[:, 1] -= BILLET_Y_OFFSET   # y now in [-ROD_Y/2, +ROD_Y/2]
# Z stays at [0, ROD_Z] so billet bottom = 0 = ANVIL_TOP_Z ✓

print(f"  Nodes: {mesh.points.shape[0]}, Elements: {mesh.cells.shape[0]}")
print(f"  X: [{mesh.points[:,0].min():.4f}, {mesh.points[:,0].max():.4f}]")
print(f"  Y: [{mesh.points[:,1].min():.4f}, {mesh.points[:,1].max():.4f}]")
print(f"  Z: [{mesh.points[:,2].min():.4f}, {mesh.points[:,2].max():.4f}]")
print()

# ============================================================
# [3] MATERIAL MODEL  –  J2 Elastoplasticity with hardening
# ============================================================
# Architecture: follows official JAX-FEM plasticity example.
# get_tensor_map() → stress_return_map(u_grad, sigma_old, eps_old)
# Internal vars: [sigmas_old, epsilons_old, eps_p_eq_old]
# ============================================================

def _lame(E, nu):
    mu  = E / (2.0 * (1.0 + nu))
    lam = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    return mu, lam

_MU, _LAM = _lame(STEEL_E, STEEL_NU)


class ForgingPlasticity(Problem):
    """J2 elastoplastic problem with linear isotropic hardening.

    Internal variables:
        [0] sigmas_old   (n_cells, n_quads, 3, 3)
        [1] epsilons_old (n_cells, n_quads, 3, 3)
        [2] eps_p_eq_old (n_cells, n_quads)       – equivalent plastic strain
    """

    def custom_init(self):
        self.fe = self.fes[0]
        nc  = len(self.fe.cells)
        nq  = self.fe.num_quads
        dim = self.dim

        self.sigmas_old   = np.zeros((nc, nq, dim, dim))
        self.epsilons_old = np.zeros((nc, nq, dim, dim))
        self.eps_p_eq_old = np.zeros((nc, nq))

        # JAX-FEM reads internal_vars as a list; order must match
        # the extra args after u_grad in stress_return_map.
        self.internal_vars = [
            self.sigmas_old,
            self.epsilons_old,
            self.eps_p_eq_old,
        ]

    # ── tensor map (called per quadrature point by JAX-FEM) ──
    def get_tensor_map(self):
        _, stress_return_map = self._get_constitutive_maps()
        return stress_return_map

    def _get_constitutive_maps(self):
        mu  = _MU
        lam = _LAM
        sig0 = float(STEEL_YIELD)
        H    = float(STEEL_HARDENING)   # linear isotropic hardening modulus
        dim  = 3

        def safe_sqrt(x):
            return jnp.where(x > 0.0, jnp.sqrt(x), 0.0)

        def safe_div(x, y):
            return jnp.where(y == 0.0, 0.0, x / y)

        def strain_from_grad(u_grad):
            return 0.5 * (u_grad + u_grad.T)

        def elastic_stress(eps):
            return lam * jnp.trace(eps) * jnp.eye(dim) + 2.0 * mu * eps

        def stress_return_map(u_grad, sigma_old, epsilon_old, eps_p_eq_old):
            """Incremental J2 radial-return.

            Args:
                u_grad      : (3,3) current displacement gradient
                sigma_old   : (3,3) stress at previous increment
                epsilon_old : (3,3) total strain at previous increment
                eps_p_eq_old: scalar equiv. plastic strain at previous inc.

            Returns:
                sigma_new   : (3,3) updated stress
            """
            eps_crt   = strain_from_grad(u_grad)
            delta_eps = eps_crt - epsilon_old
            sigma_tr  = sigma_old + elastic_stress(delta_eps)

            # deviatoric trial stress
            s_dev   = sigma_tr - (1.0 / dim) * jnp.trace(sigma_tr) * jnp.eye(dim)
            s_norm  = safe_sqrt(1.5 * jnp.sum(s_dev * s_dev))

            # current yield stress (isotropic hardening)
            sig_y   = sig0 + H * eps_p_eq_old

            # yield criterion
            f_yield = s_norm - sig_y

            # plastic multiplier (only positive part)
            d_gamma = jnp.where(f_yield > 0.0,
                                f_yield / (2.0 * mu + (2.0 / 3.0) * H),
                                0.0)

            # corrected stress
            n_hat   = safe_div(s_dev, s_norm)     # unit normal
            sigma_new = sigma_tr - 2.0 * mu * d_gamma * n_hat

            return sigma_new

        return strain_from_grad, stress_return_map

    # ── helper to update internal state after each solve ──────
    def stress_strain_fns(self):
        strain_fn, srm = self._get_constitutive_maps()
        vmap_strain = jax.vmap(jax.vmap(strain_fn))

        # vmap over cells and quadrature points
        vmap_srm = jax.vmap(jax.vmap(srm))
        return vmap_strain, vmap_srm

    def update_internal_vars(self, sol):
        """Advance sigmas_old, epsilons_old, eps_p_eq_old."""
        u_grads = self.fe.sol_to_grad(sol)
        vmap_strain, vmap_srm = self.stress_strain_fns()

        new_sigma   = vmap_srm(
            u_grads,
            self.sigmas_old,
            self.epsilons_old,
            self.eps_p_eq_old,
        )
        new_epsilon = vmap_strain(u_grads)

        # equivalent plastic strain increment
        mu  = _MU
        H   = float(STEEL_HARDENING)
        sig0 = float(STEEL_YIELD)

        delta_eps = new_epsilon - self.epsilons_old
        sigma_tr  = self.sigmas_old + (
            _LAM * jnp.trace(delta_eps, axis1=-2, axis2=-1)[..., None, None]
            * jnp.eye(3)
            + 2.0 * mu * delta_eps
        )
        s_dev  = sigma_tr - (1.0/3.0) * jnp.trace(sigma_tr, axis1=-2, axis2=-1)[..., None, None] * jnp.eye(3)
        s_norm = jnp.sqrt(jnp.maximum(1.5 * jnp.sum(s_dev * s_dev, axis=(-2, -1)), 0.0))
        sig_y  = sig0 + H * self.eps_p_eq_old
        f_y    = s_norm - sig_y
        d_gamma = jnp.where(f_y > 0.0,
                            f_y / (2.0 * mu + (2.0/3.0) * H),
                            0.0)
        new_eps_p_eq = self.eps_p_eq_old + (jnp.sqrt(2.0/3.0) * 2.0 * mu / (2.0 * mu + (2.0/3.0) * H)) * d_gamma

        self.sigmas_old   = np.array(new_sigma)
        self.epsilons_old = np.array(new_epsilon)
        self.eps_p_eq_old = np.array(new_eps_p_eq)

        self.internal_vars = [
            self.sigmas_old,
            self.epsilons_old,
            self.eps_p_eq_old,
        ]

    def get_equiv_plastic_strain(self):
        """Return (n_cells, n_quads) equiv. plastic strain array."""
        return self.eps_p_eq_old

    def get_von_mises(self):
        """Return (n_cells, n_quads) von Mises stress."""
        s = self.sigmas_old
        s_dev = s - (1.0/3.0) * np.trace(s, axis1=-2, axis2=-1)[..., None, None] * np.eye(3)
        return np.sqrt(1.5 * np.sum(s_dev * s_dev, axis=(-2, -1)))


# ============================================================
# [4] BOUNDARY CONDITION HELPERS
# ============================================================
#
# JAX-FEM Dirichlet BCs are tuples:
#   location_fn(point) → bool
#   vec               → which displacement component (0=x,1=y,2=z)
#   value_fn(point)   → float
#
# Strategy per pass:
#   - Bottom face (Z=0): fixed in Z (anvil support). Free X, Y.
#   - Active die region top face (Z=ROD_Z, X in die range):
#     prescribed downward Z displacement.
#   - The die region top nodes are rotated in the XY plane
#     (Y/Z for the cross-section) to simulate die rotation.
#
# ROTATION MAPPING:
#   For pass angle θ around the rod axis (X), a face that was
#   originally pressed in -Z is now pressed in direction
#   (0, -sin θ, -cos θ). We decompose this into Y and Z
#   Dirichlet BCs at the active top/side faces.
#
#   For a flat die pressing a flat face we need nodes on the
#   face perpendicular to (sin θ, cos θ) in YZ, i.e. the
#   face at distance apothem from centre in that direction.
#   We select nodes using a tolerance band.
# ============================================================

ATOL = 1e-6        # geometric tolerance
NODE_BAND = 1.5e-3 # 1.5 mm tolerance band for face selection


def make_bc_info(
    phase,          # "A" or "B"
    angle_deg,      # rotation angle in degrees (0=top face)
    frac,           # load fraction 0→1 (press) or 1→0 (release)
    pts             # node coordinates (N,3) — needed to compute displaced Y
):
    """
    Build dirichlet_bc_info for one load increment.

    Returns (location_fns, vecs, value_fns).

    BC structure:
      1. Z=0 (anvil face): z-fixed to 0.
      2. Active die face: displace inward by frac * stroke.

    The die face is the billet surface in direction
    (0, sin θ, cos θ) from the rod axis. For θ=0 that is
    the top face (Z=ROD_Z). For θ=60° it is the face at
    a 60° rotation around X.

    We parameterise by which billet surface to select.
    The 6 passes hit the 6 hex faces; we identify each face
    by its normal in the YZ cross-section.
    """
    theta = math.radians(angle_deg)

    # Face normal in YZ (the face the die presses):
    #   ny = sin(theta), nz = cos(theta)
    ny = math.sin(theta)
    nz = math.cos(theta)

    # Distance from rod centreline in this direction
    # (half-width of billet in that direction):
    #   For a 4"×4" square billet, the inscribed circle radius = 2"
    #   and any face is at distance 2" = ROD_Z/2 from centre in Z,
    #   or ROD_Y/2 from centre in Y.
    # We compute dot product of (y - y_centre, z - z_centre) with (ny, nz)
    # and select nodes near the maximum = ROD_Z/2.
    y_centre = 0.0        # billet Y centre after mesh shift
    z_centre = ROD_Z / 2.0

    # Maximum projection distance (face distance from centre)
    # For a 4"×4" square, the face distance for angle θ is:
    #   d_face = min(ROD_Z/2 / |cos θ|, ROD_Y/2 / |sin θ|)  for corner angles
    # But since we always press flat against one of the 6 hex faces we use
    # ROD_Z/2 as initial billet half-width (both Y and Z are 4").
    d_face = ROD_Z / 2.0   # = ROD_Y / 2.0 since square billet

    # Stroke (how far die travels per pass)
    if phase == "A":
        target_apothem = HEX_APOTH_A
        x0, x1         = DIE_A_X0, DIE_A_X1
    else:
        target_apothem = HEX_APOTH_B
        x0, x1         = DIE_B_X0, DIE_B_X1

    # After previous passes the face is already at the apothem of the
    # PREVIOUS pass; for pass 0 of Phase A it starts at d_face = ROD_Z/2.
    # We track cumulative stroke simply as:
    #   total displacement = d_face - target_apothem  (first pass moves all of it)
    #   Subsequent passes: material already near apothem, stroke ~ 0.
    # A more physical approach: prescribe final position as target_apothem,
    # and ramp frac from 0→1 over N_INCREMENTS. JAX-FEM handles the
    # nonlinear load redistribution.
    stroke = d_face - target_apothem   # > 0 always

    disp_mag = frac * stroke   # current prescribed inward displacement

    # ── Anvil BC: bottom face fixed in Z ─────────────────────

    #def bottom_z(point):
        #return np.isclose(point[2], 0.0, atol=ATOL)
    
    def bottom_z(point):
        return jnp.abs(point[2]) < ATOL

    def zero_disp(point):
        return 0.0

    # ── Die BC: top/side face of billet in active X region ───
    # Select nodes: in X range AND on the billet surface facing
    # the die (projection > d_face - NODE_BAND).
    
    #def die_face_loc(point):
        #in_x = (point[0] >= x0 - ATOL) and (point[0] <= x1 + ATOL)
        #proj = ny * (point[1] - y_centre) + nz * (point[2] - z_centre)
        #on_face = proj >= (d_face - NODE_BAND)
        #return in_x and on_face

    # Displacement components in Y and Z (die pushes inward
    # along -ny, -nz direction by disp_mag)
    def die_disp_y(point):
        # Displace in -ny direction: dy = -ny * disp_mag
        return -ny * disp_mag

    def die_disp_z(point):
        return -nz * disp_mag

    # Build BC info
    location_fns = [bottom_z, die_face_loc, die_face_loc]
    vecs         = [2,        1,            2         ]
    value_fns    = [zero_disp, die_disp_y,  die_disp_z]

    # Also fix X displacement of the bottom face ends to prevent
    # rigid body motion in X:
    #def left_end(point):
       # return np.isclose(point[0], 0.0, atol=ATOL)

    #def right_end(point):
        #return np.isclose(point[0], ROD_LEN, atol=ATOL)
    

    def left_end(point):
        return jnp.abs(point[0]) < ATOL

    def right_end(point):
        return jnp.abs(point[0] - ROD_LEN) < ATOL

    location_fns += [left_end,  right_end]
    vecs         += [0,          0        ]
    value_fns    += [zero_disp,  zero_disp]

    return [location_fns, vecs, value_fns]


# ============================================================
# [5] PROBLEM FACTORY
# ============================================================

def make_problem(bc_info):
    """Instantiate a fresh ForgingPlasticity problem.

    We re-create the problem each time BC locations change
    (new die region / angle). Internal variables are preserved
    by copying them over after creation.
    """
    prob = ForgingPlasticity(
        mesh=mesh,
        vec=3,
        dim=3,
        ele_type=ele_type,
        dirichlet_bc_info=bc_info,
    )
    return prob


# ============================================================
# [6] FORGING PASS RUNNER
# ============================================================

def run_forging_pass(problem, phase, angle_deg, pass_idx,
                     n_increments, frame_list):
    """
    Run one forging pass (press + release) incrementally.

    Modifies problem.sigmas_old / epsilons_old / eps_p_eq_old
    in place via update_internal_vars().

    Appends visualisation dicts to frame_list.
    """
    print(f"  [Pass {pass_idx}] phase={phase} angle={angle_deg:.0f}°")
    t0 = time.time()

    # Press phase (frac: 0 → 1)
    for inc in range(n_increments + 1):
        frac = inc / n_increments

        bc_info = make_bc_info(
            phase=phase,
            angle_deg=angle_deg,
            frac=frac,
            pts=mesh.points,
        )
        # Update BCs on existing problem
        problem.fe.update_Dirichlet_boundary_conditions(bc_info)

        try:
            sol_list = solver(
                problem,
                solver_options={"petsc_solver": {}},
            )
        except Exception as e:
            print(f"    [WARNING] Solver failed at inc={inc}, frac={frac:.2f}: {e}")
            # Continue with previous state
            continue

        sol = sol_list[0]
        problem.update_internal_vars(sol)

        # Capture frame at 0%, 50%, 100% press
        if inc in (0, n_increments // 2, n_increments):
            pts_displaced = mesh.points + np.array(sol).reshape(-1, 3)
            frame_list.append({
                "pts":      pts_displaced,
                "eps_p":    problem.eps_p_eq_old.copy(),
                "vm":       problem.get_von_mises().copy(),
                "sigma":    problem.sigmas_old.copy(),
                "pass_idx": pass_idx,
                "phase":    phase,
                "angle":    angle_deg,
                "frac":     frac,
                "sub":      "press",
            })

        # Save VTK
        vtk_name = f"pass{pass_idx:02d}_{phase}_a{angle_deg:.0f}_inc{inc:02d}.vtu"
        save_sol(problem.fe, sol,
                 os.path.join(VTK_DIR, vtk_name))

    # Release phase (frac: 1 → 0) — 5 steps
    for inc in range(5 + 1):
        frac = 1.0 - inc / 5

        bc_info = make_bc_info(
            phase=phase,
            angle_deg=angle_deg,
            frac=frac,
            pts=mesh.points,
        )
        problem.fe.update_Dirichlet_boundary_conditions(bc_info)

        try:
            sol_list = solver(
                problem,
                solver_options={"petsc_solver": {}},
            )
        except Exception as e:
            print(f"    [WARNING] Release solver failed at inc={inc}: {e}")
            continue

        sol = sol_list[0]
        problem.update_internal_vars(sol)

    # Capture one frame at release
    pts_displaced = mesh.points + np.array(sol_list[0]).reshape(-1, 3)
    frame_list.append({
        "pts":      pts_displaced,
        "eps_p":    problem.eps_p_eq_old.copy(),
        "vm":       problem.get_von_mises().copy(),
        "sigma":    problem.sigmas_old.copy(),
        "pass_idx": pass_idx,
        "phase":    phase,
        "angle":    angle_deg,
        "frac":     0.0,
        "sub":      "released",
    })

    elapsed = time.time() - t0
    ep_max  = problem.eps_p_eq_old.max()
    print(f"    Done in {elapsed:.1f}s  max_eps_p={ep_max:.4f}")
    return sol_list[0]


# ============================================================
# [7] ANIMATION RENDERER
# ============================================================

def hex_outline_yz(apothem, cz_centre, cumul_angle_deg):
    """Return (ys, zs) of a hex outline in the YZ plane."""
    r = apothem / math.cos(math.radians(30.0))
    ys, zs = [], []
    for i in range(7):
        a = math.radians(cumul_angle_deg + i * 60.0 + 30.0)
        ys.append(r * math.sin(a))
        zs.append(cz_centre + r * math.cos(a))
    return ys, zs


def draw_box_2d_xz(ax, cx, cz, dx, dz, fc, ec, alpha, label=""):
    rect = plt.Rectangle(
        (cx - dx/2, cz - dz/2), dx, dz,
        linewidth=1.4, edgecolor=ec, facecolor=fc,
        alpha=alpha, zorder=3)
    ax.add_patch(rect)
    if label:
        ax.text(cx, cz, label, ha="center", va="center",
                color=ec, fontsize=7, fontweight="bold", zorder=4)


def draw_box_3d(ax, cx, cy, cz, dx, dy, dz, fc, alpha, ec):
    x0, x1 = cx - dx/2, cx + dx/2
    y0, y1 = cy - dy/2, cy + dy/2
    z0, z1 = cz - dz/2, cz + dz/2
    verts = [
        [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0)],
        [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)],
        [(x0,y1,z0),(x1,y1,z0),(x1,y1,z1),(x0,y1,z1)],
        [(x0,y0,z0),(x0,y1,z0),(x0,y1,z1),(x0,y0,z1)],
        [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],
    ]
    poly = Poly3DCollection(verts, alpha=alpha,
                            facecolors=fc, edgecolors=ec, linewidths=0.7)
    ax.add_collection3d(poly)


def fig_to_bgr(fig, W, H):
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    buf  = np.frombuffer(fig.canvas.buffer_rgba(), np.uint8).reshape(h, w, 4)
    bgr  = cv2.cvtColor(buf[:, :, :3], cv2.COLOR_RGB2BGR)
    if bgr.shape[1] != W or bgr.shape[0] != H:
        bgr = cv2.resize(bgr, (W, H), cv2.INTER_AREA)
    return bgr


def render_frame(frame_data, frame_idx):
    """
    Render one simulation frame to BGR image (matching Genesis style).

    Layout:
      Top-left : 3D convex hull (equiv. plastic strain)
      Top-mid  : YZ cross-section with hex targets
      Top-right: XZ side view with die + anvil
      Bottom   : Diagnostics info strip
    """
    pts   = frame_data["pts"]          # (N_nodes, 3) displaced coords
    eps_p = frame_data["eps_p"]        # (n_cells, n_quads)
    vm    = frame_data["vm"]           # (n_cells, n_quads)
    pid   = frame_data["pass_idx"]
    phase = frame_data["phase"]
    angle = frame_data["angle"]
    frac  = frame_data["frac"]
    sub   = frame_data["sub"]

    # Reduce per-node for scatter: use node average of neighbour cell values
    # Simple: just use node positions from displaced mesh; colour by Z coord
    # as proxy (or by a per-node interpolated eps_p).
    # Here we colour nodes by their Z height (proxy for compression depth).
    # For a richer view we use the cell centroid eps_p scattered over nodes.

    # Get cell centroids (average of 8 HEX8 nodes)
    n_cells = mesh.cells.shape[0]
    cell_pts  = pts[mesh.cells.reshape(-1)].reshape(n_cells, 8, 3)
    centroids = cell_pts.mean(axis=1)         # (n_cells, 3)
    cell_eps  = eps_p.mean(axis=-1)           # (n_cells,) mean over quads
    cell_vm   = vm.mean(axis=-1) / 1e6        # MPa

    eps_max = max(float(cell_eps.max()), 1e-8)
    vm_max  = max(float(cell_vm.max()), 0.1)

    # ── Die geometry for this frame ───────────────────────────
    die_cx     = DIE_A_CX if phase == "A" else DIE_B_CX
    die_len    = DIE_A_LEN if phase == "A" else DIE_B_LEN
    target_ap  = HEX_APOTH_A if phase == "A" else HEX_APOTH_B
    theta      = math.radians(angle)
    ny_die     = math.sin(theta)
    nz_die     = math.cos(theta)
    stroke     = ROD_Z / 2.0 - target_ap
    # Die face centre in 3D
    die_face_z = ROD_Z / 2.0 + (ROD_Z/2.0 - frac * stroke) * nz_die
    die_face_y = (ROD_Z / 2.0 - frac * stroke) * ny_die
    die_cz_vis_curr = die_face_z + DIE_THICKNESS / 2.0 * nz_die
    die_cy_vis_curr = die_face_y + DIE_THICKNESS / 2.0 * ny_die

    # ── Figure ────────────────────────────────────────────────
    fig = plt.figure(figsize=(ANIM_W/80, ANIM_H/80), dpi=80)
    fig.patch.set_facecolor("#0d0d0d")

    gs_fig = fig.add_gridspec(
        2, 3,
        height_ratios=[8, 1.2],
        hspace=0.18, wspace=0.12,
        left=0.04, right=0.97, top=0.92, bottom=0.06
    )
    ax3D = fig.add_subplot(gs_fig[0, 0], projection="3d")
    axYZ = fig.add_subplot(gs_fig[0, 1])
    axXZ = fig.add_subplot(gs_fig[0, 2])
    axI  = fig.add_subplot(gs_fig[1, :])

    # ── 3D convex hull (equiv. plastic strain) ────────────────
    ax3D.set_facecolor("#0d0d0d")
    norm_eps = mcolors.Normalize(vmin=0, vmax=eps_max)
    cmap_eps = plt.cm.plasma

    try:
        if len(centroids) >= 4:
            hull = ConvexHull(centroids)
            hull_verts  = []
            hull_colors = []
            for s in hull.simplices:
                hull_verts.append([tuple(centroids[i]) for i in s])
                ep_mean = float(cell_eps[s].mean())
                hull_colors.append(cmap_eps(norm_eps(ep_mean)))
            ax3D.add_collection3d(Poly3DCollection(
                hull_verts, facecolors=hull_colors,
                edgecolors="none", alpha=0.88))
    except Exception:
        sc = ax3D.scatter(
            centroids[:,0], centroids[:,1], centroids[:,2],
            c=cell_eps, cmap="plasma", vmin=0, vmax=eps_max,
            s=3, alpha=0.85, linewidths=0)

    # Anvil (grey)
    draw_box_3d(ax3D, billet_cx, 0.0, anvil_cz_vis,
                ROD_LEN * 2.0, DIE_WIDTH_Y * 2.0, ANVIL_THICKNESS,
                "#b0b8c8", 0.55, "#d0d8e8")
    # Die (steel blue)
    draw_box_3d(ax3D, die_cx, die_cy_vis_curr, die_cz_vis_curr,
                die_len, DIE_WIDTH_Y, DIE_THICKNESS,
                "#3a7acc", 0.55, "#7ab0ff")

    ax3D.set_xlim(-0.01, ROD_LEN + 0.01)
    ax3D.set_ylim(-ROD_Y * 1.6, ROD_Y * 1.6)
    ax3D.set_zlim(anvil_cz_vis - ANVIL_THICKNESS, billet_top + DIE_THICKNESS + 0.01)
    ax3D.view_init(22, -48)
    ax3D.set_title("3D Billet – Equiv. Plastic Strain",
                   color="white", fontsize=9, fontweight="bold", pad=4)
    for fn, lbl in [(ax3D.set_xlabel, "X(m)"),
                    (ax3D.set_ylabel, "Y(m)"),
                    (ax3D.set_zlabel, "Z(m)")]:
        fn(lbl, color="#999999", fontsize=6, labelpad=3)
    ax3D.tick_params(colors="#666666", labelsize=5)
    for pane in (ax3D.xaxis.pane, ax3D.yaxis.pane, ax3D.zaxis.pane):
        pane.fill = False; pane.set_edgecolor("#222222")

    sm = plt.cm.ScalarMappable(cmap=cmap_eps, norm=norm_eps)
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax3D, shrink=0.55, pad=0.10, aspect=22)
    cb.set_label("ε_p eq", color="white", fontsize=7)
    plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color="white")

    # ── YZ cross-section (X ≈ mid-die region) ────────────────
    axYZ.set_facecolor("#111111")
    axYZ.set_aspect("equal")

    # Select nodes in thin X slice near die centre
    x_slice = np.abs(pts[:, 0] - die_cx) < 2.5e-3
    p_yz = pts[x_slice] if x_slice.sum() > 5 else pts
    # Colour by Z height (proxy for compression)
    c_yz = p_yz[:, 2]
    sc_yz = axYZ.scatter(p_yz[:, 1], p_yz[:, 2],
                         c=c_yz, cmap="coolwarm", s=5, alpha=0.80,
                         linewidths=0)

    # Target hex overlays
    billet_cz_dyn = (pts[:, 2].max() + pts[:, 2].min()) / 2.0
    if phase == "B":
        ys_a, zs_a = hex_outline_yz(HEX_APOTH_A, billet_cz_dyn, 0.0)
        axYZ.plot(ys_a, zs_a, "--", color="#44aaff", lw=1.0, alpha=0.7,
                  label="target hex A (4.0\" AF)")
        ys_b, zs_b = hex_outline_yz(HEX_APOTH_B, billet_cz_dyn, 0.0)
        axYZ.plot(ys_b, zs_b, "-",  color="#ff9944", lw=1.4, alpha=0.9,
                  label="target hex B (2.75\" AF)")
    else:
        cumu = (pid % N_PASSES_A) * 60.0
        ys_a, zs_a = hex_outline_yz(HEX_APOTH_A, billet_cz_dyn, cumu)
        axYZ.plot(ys_a, zs_a, "--", color="#44aaff", lw=1.4, alpha=0.85,
                  label="target hex A (4.0\" AF)")

    axYZ.legend(fontsize=7, facecolor="#222222", edgecolor="#444444",
                labelcolor="white", loc="upper right")
    axYZ.set_xlim(-ROD_Y * 1.3, ROD_Y * 1.3)
    axYZ.set_ylim(-0.002, billet_top + 0.02)
    axYZ.set_title(f"YZ cross-section  (X≈{die_cx*1e3:.0f}mm)",
                   color="white", fontsize=9, fontweight="bold")
    axYZ.set_xlabel("Y (m)", color="#888888", fontsize=7)
    axYZ.set_ylabel("Z (m)", color="#888888", fontsize=7)
    axYZ.tick_params(colors="#666666", labelsize=6)
    for sp in axYZ.spines.values(): sp.set_edgecolor("#333333")
    fig.colorbar(sc_yz, ax=axYZ, shrink=0.6, pad=0.04, aspect=18).set_label(
        "Z (m)", color="white", fontsize=7)

    # ── XZ side view ─────────────────────────────────────────
    axXZ.set_facecolor("#111111")
    axXZ.set_aspect("equal")
    # Project all nodes to XZ
    c_xz = cell_eps   # colour centroids by eps_p
    sc_xz = axXZ.scatter(centroids[:, 0], centroids[:, 2],
                         c=c_xz, cmap="plasma", vmin=0, vmax=eps_max,
                         s=4, alpha=0.75, linewidths=0)
    # Die
    die_xz_cz = ROD_Z / 2.0 + (ROD_Z / 2.0 + DIE_THICKNESS / 2.0 - frac * ROD_Z / 2.0 + frac * target_ap)
    draw_box_2d_xz(axXZ, die_cx, billet_top + DIE_THICKNESS / 2.0,
                   die_len, DIE_THICKNESS,
                   "#1a4a88", "#4499ff", 0.55, "DIE")
    # Anvil
    draw_box_2d_xz(axXZ, billet_cx, anvil_cz_vis,
                   ROD_LEN * 2.0, ANVIL_THICKNESS,
                   "#4a5060", "#a0a8b8", 0.60, "ANVIL")
    # Phase zone markers
    axXZ.axvline(DIE_A_X0, color="#44aaff", lw=0.7, ls="--", alpha=0.7)
    axXZ.axvline(DIE_A_X1, color="#44aaff", lw=0.7, ls="--", alpha=0.7)
    axXZ.axvline(DIE_B_X1, color="#ff9944", lw=0.7, ls="--", alpha=0.7)

    axXZ.set_xlim(-0.01, ROD_LEN + 0.01)
    axXZ.set_ylim(anvil_cz_vis - ANVIL_THICKNESS / 2 - 0.005,
                  billet_top + DIE_THICKNESS + 0.015)
    axXZ.set_title("XZ side view – die zones",
                   color="white", fontsize=9, fontweight="bold")
    axXZ.set_xlabel("X (m)", color="#888888", fontsize=7)
    axXZ.set_ylabel("Z (m)", color="#888888", fontsize=7)
    axXZ.tick_params(colors="#666666", labelsize=6)
    for sp in axXZ.spines.values(): sp.set_edgecolor("#333333")
    fig.colorbar(sc_xz, ax=axXZ, shrink=0.6, pad=0.04, aspect=18).set_label(
        "ε_p eq", color="white", fontsize=7)

    # ── Info strip ───────────────────────────────────────────
    axI.set_facecolor("#111111"); axI.axis("off")
    billet_H = (pts[:, 2].max() - pts[:, 2].min()) * 1e3
    billet_W = (pts[:, 1].max() - pts[:, 1].min()) * 1e3
    billet_L = (pts[:, 0].max() - pts[:, 0].min()) * 1e3

    axI.text(0.5, 0.5,
        f"frame={frame_idx:04d}  |  Phase {phase}  pass {pid}  "
        f"angle={angle:.0f}°  [{sub.upper()}]  frac={frac:.2f}  |"
        f"  L={billet_L:.1f}mm  H={billet_H:.1f}mm  W={billet_W:.1f}mm  |"
        f"  ε_p_max={float(eps_p.max()):.4f}  σ_VM_max={float(vm.max())/1e6:.1f}MPa"
        f"  N_nodes={pts.shape[0]}",
        ha="center", va="center",
        color="#44ee66", fontsize=8, fontfamily="monospace",
        transform=axI.transAxes,
        bbox=dict(facecolor="#1a1a1a", edgecolor="#333333",
                  boxstyle="round,pad=0.4"))

    fig.suptitle(
        f"HEX ROD FORGING (JAX-FEM)  ·  Phase {phase}  pass {pid}  "
        f"·  angle={angle:.0f}°  ·  {sub.upper()}  ·  frac={frac:.2f}",
        color="white", fontsize=10, fontweight="bold", y=0.97)

    bgr = fig_to_bgr(fig, ANIM_W, ANIM_H)
    plt.close(fig)
    return bgr


# ============================================================
# [8] MAIN SIMULATION LOOP
# ============================================================

print("="*65)
print("  STARTING FORGING SIMULATION")
print("="*65)
print()

# Initial BC (all zero — just to build the problem)
initial_bc = make_bc_info("A", 0.0, 0.0, mesh.points)
problem     = make_problem(initial_bc)

all_frames  = []   # list of frame_data dicts
frame_imgs  = []   # list of BGR images for video

# ── Phase A: 6 passes at 0°, 60°, 120°, 180°, 240°, 300° ────
print("  ━━━━ PHASE A ━━━━")
for local_pass in range(N_PASSES_A):
    angle   = local_pass * 60.0
    pass_id = local_pass
    sol = run_forging_pass(
        problem, "A", angle, pass_id,
        N_INCREMENTS, all_frames
    )

print()
print("  ━━━━ PHASE B ━━━━")
# ── Phase B: 6 passes at 0°, 60°, … ─────────────────────────
for local_pass in range(N_PASSES_B):
    angle   = local_pass * 60.0
    pass_id = N_PASSES_A + local_pass
    sol = run_forging_pass(
        problem, "B", angle, pass_id,
        N_INCREMENTS, all_frames
    )

print()
print(f"  Simulation complete. Total frames captured: {len(all_frames)}")

# ============================================================
# RENDER ALL FRAMES & WRITE VIDEO
# ============================================================
print()
print("  Rendering animation frames …")

video = cv2.VideoWriter(
    VIDEO_PATH,
    cv2.VideoWriter_fourcc(*"mp4v"),
    FPS,
    (ANIM_W, ANIM_H)
)

for fi, fd in enumerate(all_frames):
    bgr = render_frame(fd, fi)
    video.write(bgr)
    png_path = os.path.join(FRAME_DIR, f"frame_{fi:04d}.png")
    cv2.imwrite(png_path, bgr)
    if fi % 5 == 0:
        print(f"    Frame {fi}/{len(all_frames)} rendered")

video.release()
print()
print(f"  Video saved: {VIDEO_PATH}")
print(f"  Frames saved: {FRAME_DIR}/")

# ============================================================
# FINAL SUMMARY
# ============================================================
print()
print("="*65)
print("  SIMULATION COMPLETE")
print("="*65)
print()
print("  EXPECTED OUTCOMES:")
print("  0\"–16\"  : rectangular stock (no BC pressure)")
print("  16\"–20\" : large hex ~4.0\" AF (Phase A, 6 passes at 60° increments)")
print("  20\"–24\" : small hex ~2.75\" AF (Phase B, 6 passes at 60° increments)")
print()
print("  NOTE: Hexagonal geometry is NOT analytically enforced.")
print("        It emerges from incremental elastoplastic deformation.")
print("        Each pass compresses one face of the progressively")
print("        forming hexagon. The plastic strain field shows")
print("        highest values under die contact regions.")
print()
print("  OUTPUTS:")
print(f"    {VIDEO_PATH}")
print(f"    {VTK_DIR}/ (VTU files for ParaView)")
print(f"    {FRAME_DIR}/ (individual PNG frames)")
print("="*65)