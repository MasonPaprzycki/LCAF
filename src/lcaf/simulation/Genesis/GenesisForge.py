#########################################################################
# GENESIS 1.1 FORGING SIMULATION  
##########################################################################
# Designed for constant temperature hot billet upset between punch and anvil
# Variable temperature feedback would need an updated architecture. 
#
# This is a very slow and computationally expensive simulation process. 
# To run reasonably on a average work station we needed to use a pretty low resolution. 
#
# Further more, forging temperature steel is still very stiff and the numerical complexity is quite high
# We ran into large instability issues where the simulation would blow up.
# So we introduced a remeshing strategy to avoid instability from excited particles.
# This is only a concern during very large deformation though. 
# 
# Also note that this simulation is yet to be experimentally validated.
#
##### BUG HISTORY #######################################################
#
# BUG 1 (original): gs.morphs.Box + gs.materials.Rigid()
#   Box morphs with Rigid material have n_dofs = 0.
#   They are permanently frozen in space.  set_dofs_velocity()
#   is a silent no-op.  The MPM coupler reads v_rigid = 0
#   every substep → zero contact impulse → J stays at 1.0.
#
# BUG 2 (v2 attempt): gs.morphs.Box + gs.materials.Tool()
#   Tool material ALSO requires a Mesh morph, not Box.
#   ToolEntity.__init__ calls morph.scale which Box does not
#   expose → AttributeError: 'Box' object has no attribute#   'scale' → crash before scene.build().
#
# BUG 3: Billet must start levitated above the interface of the
#   anvil by a distance of about half a particle. Otherwise at
#   the interface the particles will collide or be adjusted
#   and you will consequently see high velocities and weird
#   perfectly flat bars of high strain along frame zero.
#   FIX: INIT_GAP = PARTICLE_SIZE * 0.5 applied on EVERY
#   set-down, including after rotation. This is now enforced
#   inside BilletState.compute_setdown_center_z() and
#   remesh_scene() so it can never be forgotten.
#
# BUG 4 (v4→v5): gs.init() raises GenesisException("Genesis
#   already initialized.") on every remesh after the first
#   because Genesis 1.1 is NOT idempotent across calls to
#   gs.init() within a single Python process.
#   FIX: gs_init_fresh() calls gs.destroy() first whenever
#   Genesis is already live, then calls gs.init().  A module-
#   level flag _genesis_initialized tracks live state so the
#   destroy is only issued when actually needed.
#
#
# ####### Remeshing design ####################################
#   Remeshing tears down the Genesis scene completely and
#   starts a brand-new simulation with fresh particle state.
#   The current DEFORMED GEOMETRY is preserved by sizing the
#   new billet Box morph to match the AABB of the billet
#   at the moment of remesh.  Particles are seeded fresh on
#   a clean uniform grid inside that volume — zero velocity,
#   identity deformation gradient, correct density.
#
#   This is a terrible simulation design but since were using a research 
#   grade simulation the most practical approach is to just ensure no numerical 
#   blow up and take our results with a grain of salt. Were using it 
#   for feedback informed closed loop so it shouldnt be a big deal. 
#
#   Procedure:
#     1. Read particle positions from billet.get_state()
#        to determine current AABB (deformed geometry).
#     2. Call gs_init_fresh() to destroy the old Genesis
#        runtime and start a clean one.
#     3. Recreate scene with billet Box sized to AABB.
#     4. scene.build() seeds fresh particles inside AABB.
#     5. particles start at rest with F=I.
#
# ################ Rotation design ####################################
#   Rotation is handled entirely in Python before remeshing.
#   No Genesis "pick up" API is used — instead:
#     1. BilletState stores current particle positions (world)
#        and a running cumulative rotation matrix R_cumulative.
#     2. "Pick up" = translate positions so billet center is
#        at origin, apply rotation matrix, translate back to
#        a new resting position above the anvil (with INIT_GAP).
#     3. The rotated AABB is passed to remesh_scene() which
#        sizes the new billet Box morph accordingly.
#   Animation: frames jump instantaneously between the pre-
#   and post-rotation states. No intermediate frames are
#   rendered during the rotation move itself (as specified).
#
# ################ Tool path prescription #########################
#   Build a list of ToolPathStep objects:
#     ToolPathStep(kind="punch",    strike_cfg=StrikeConfig(...))
#     ToolPathStep(kind="rotate",   angle_deg=90, axis="x")
#   The main loop iterates steps, tracking:
#     rotation_idx  – incremented each time a rotate step runs
#     strike_idx    – incremented each time a punch step runs
#     remesh_idx    – reset to 0 at start of each punch strike
#   Frame log format:  rot{R:02d}.strike{S:02d}.remesh{K:02d}
#   Remeshing occurs:
#     • At least once BEFORE each punch strike starts
#     • punchRemeshTimes times DURING each punch strike
#       (evenly distributed across the strike's n_steps)
#     • At least once AFTER each rotation (during set-down)
#
# ############# Tool drive pattern (must be done every step) ########
#   Before every scene.step():
#     punch.set_velocity(vel=np.array([[0.0, 0.0, PUNCH_VEL_Z]]))
#     punch.set_position(pos=np.array([[0.0, 0.0, current_z]]))
#     table.set_velocity(vel=np.array([[0.0, 0.0, 0.0]]))
#     table.set_position(pos=np.array([[0.0, 0.0, table_z]]))
#
###################### Outputs ################################
#
#   output/forge_camera.mp4       – Genesis camera stream
#   output/forge_particles.mp4    – strain + Jacobian scatter
#   output/forge_solid_3d.mp4     – 3D solid hull deformation
#   output/forge_solid_xz.mp4     – XZ cross-section
#   output/forge_solid_xy.mp4     – XY footprint top view
#   output/diagnostics.csv        – per-step metrics
#   output/npy/frame_NNNNN.npy    – particle positions
#   output/particles/frame_*.png  – particle PNG stills
#   output/solid_3d/frame_*.png   – 3D solid PNG stills
#   output/solid_xz/frame_*.png   – XZ solid PNG stills
#   output/solid_xy/frame_*.png   – XY solid PNG stills
#
#############################################################

import genesis as gs

# standard python imports
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import os, csv, copy

# numerical computing packages and visualization libraries
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Rectangle as MPLRect

matplotlib.use("Agg")

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401

from scipy.spatial import ConvexHull
from scipy.ndimage import gaussian_filter

import cv2
from skimage.measure import marching_cubes


# output directorys 
for d in (
    "output",
    "output/particles",
    "output/solid_3d",
    "output/solid_xz",
    "output/solid_xy",
    "output/npy",
    "output/meshes"
):
    os.makedirs(d, exist_ok=True)


#units and geometry 
INCH = 0.0254

_genesis_initialized: bool = False   # module-level liveness flag


def gs_init_fresh() -> None:
    """
    Idempotent Genesis initialiser safe to call repeatedly.

    Destroys the current Genesis runtime if one is live, then
    calls gs.init() to start a clean instance.  Updates the
    module-level _genesis_initialized flag.

    This is the ONLY place gs.init() or gs.destroy() should
    be called anywhere in this file.
    """
    global _genesis_initialized
    if _genesis_initialized:
        print("  [genesis] Destroying previous Genesis runtime…")
        gs.destroy()
        _genesis_initialized = False
    gs.init()
    _genesis_initialized = True

@dataclass
class SolverConfig:

    # When modeling a rectangular billet 
    # with too low of a grid density the mesh begins to look like a diamond

    #Note using 500 = grid density and dt 1e-7 gives you a convicing looking mesh, 
    # using 1000 = grid density and dt = = 3.1e-8 gives you a high resolution looking mesh 
    # these simulations would become terribly slow though and we cannot practically run them 
    #increasing grid density not only drastically increases the complexity per time step
    # it also increases your number of steps becuase your time step must be smaller 

    # So for a decent simulation I think we would need grid density ~ 500 -1000 
    # At this point it may or may not be worth using MPM 


    # time integration
    dt:           float = 4e-7 #1e-7 
    substeps:     int   = 16
    n_steps:      int   = 27778       
    export_every: int   = 1000        

    # MPM grid
    grid_density: int  = 100 #500 
    enable_cpic:  bool = True

    # diagnostics
    cfl_warn:     float = 0.10
    cfl_critical: float = 1.00

    j_invert_warn:  float = 0.05
    j_explode_warn: float = 2.50

    strain_jump:        float = 5.0
    particle_vel_warn:  float = 15.0
    particle_mass_warn: float = 1e-9

@dataclass
class MaterialConfig:
    E:   float
    nu:  float
    rho: float

@dataclass
class ForgeConfig:
    billet_x: float
    billet_y: float
    billet_z: float

    anvil_x:         float
    anvil_y:         float
    anvil_thickness: float

    punch_x:         float
    punch_y:         float
    punch_thickness: float

    # simulation domain margins
    lateral_margin: float = 0.050
    vertical_margin: float = 0.030

# Strike configuration (per individual punch strike)
# Each StrikeConfig carries its own stroke profile parameters
# as well as the number of remeshes to perform for the strike
@dataclass
class StrikeConfig:
    """
    Parameters for a single punch strike.

    Attributes
    ----------
    down_dist_m : float
        Distance the punch travels downward (metres).
    up_dist_m : float
        Distance the punch travels back upward (metres).
    velocity_ms : float
        Constant speed of the punch during both strokes (m/s).
    n_steps : int
        Number of Genesis time steps for this entire strike
        (down + up combined).  Defaults to SolverConfig.n_steps
        if left as None at construction time.
    punch_remesh_times : int
        Number of additional remeshes to perform DURING the
        strike (evenly spaced across n_steps).  A mandatory
        remesh BEFORE the strike starts is always performed
        and is NOT counted here.
    """
    down_dist_m:       float = 0.0381
    up_dist_m:         float = 0.0381
    velocity_ms:       float = 0.5
    n_steps:           Optional[int] = None   # filled in from SolverConfig
    punch_remesh_times: int = 1               # intra-strike remeshes


# ############################################################
# TOOL PATH STEP
# The tool path is a list of ToolPathStep objects processed
# in order.  Two kinds are supported:
#
#   kind="punch"   – run a punch strike described by StrikeConfig
#   kind="rotate"  – pick up billet, rotate, set back down
#
# Coordinate metadata is stored explicitly here (axis, angle,
# pivot).  The simulation never figures out geometry on its
# own; all spatial decisions are prescribed in the tool path.
# ###############################################################
@dataclass
class ToolPathStep:
    """
    One step in the forging tool path.

    For kind="punch":
        strike_cfg  – StrikeConfig describing the stroke.

    For kind="rotate":
        angle_deg   – rotation angle (degrees, right-hand rule).
        axis        – "x", "y", or "z" (rotation axis name).
        pivot_world – (3,) array: world-space pivot point for
                      the rotation.  Use None to rotate about
                      the current billet centroid.

    For kind="translate":
        translate_x – distance to move along WORLD X axis (metres).
        translate_y – distance to move along WORLD Y axis (metres).
        translate_z – distance to move along WORLD Z axis (metres).
                      All three default to 0.0; set only the axes
                      you want to move.  Movement is in world space,
                      NOT billet-local space — so X always means the
                      world X axis regardless of how the billet has
                      been rotated.

    Coordinate metadata (always stored, regardless of kind):
        world_frame – human-readable description of the world
                      coordinate frame (default "Z-up, Y-forward").
    """
    kind: str                                  # "punch", "rotate", or "translate"

    # punch-specific
    strike_cfg: Optional[StrikeConfig] = None

    # rotate-specific
    angle_deg:   float = 0.0
    axis:        str   = "x"                  # "x" | "y" | "z"
    pivot_world: Optional[np.ndarray] = None  # None → billet centroid

    # translate-specific (world-space offsets in metres)
    translate_x: float = 0.0
    translate_y: float = 0.0
    translate_z: float = 0.0

    # coordinate metadata — stored explicitly, never inferred
    world_frame: str = "Z-up, Y-forward, X-right (SI metres)"


# Stores only the particle positions 
@dataclass
class BilletState:
    """
    Geometry-only billet state used between remeshes.

    Attributes
    ----------
    pos : np.ndarray, shape (N, 3)
        Particle positions (metres) — used for AABB geometry
        only.  NOT injected into the new scene.
    R_cumulative : np.ndarray, shape (3, 3)
        Cumulative rotation matrix applied to the billet since
        the start of the simulation.  Starts as identity.
        Updated each time a rotate step executes.
    rotation_idx : int
        Number of rotation steps completed so far.
    """
    pos:          np.ndarray
    R_cumulative: np.ndarray = field(
        default_factory=lambda: np.eye(3, dtype=np.float64)
    )
    rotation_idx: int = 0

    def centroid(self) -> np.ndarray:
        """World-space centroid of current particle cloud."""
        return self.pos.mean(axis=0)

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Axis-aligned bounding box of current particles.
        Returns (min_xyz, max_xyz) each shape (3,).
        """
        return self.pos.min(axis=0), self.pos.max(axis=0)

    def compute_setdown_center_z(
        self,
        anvil_top_z:   float,
        particle_size: float,
        init_gap_frac: float = 0.5
    ) -> float:
        """
        Compute the Z-centre of the billet for the next set-down
        such that the BOTTOM of the particle cloud is levitated
        above the anvil top by (init_gap_frac * particle_size).

        This enforces BUG-3 fix on every set-down so it can
        never be forgotten.

        Parameters
        ----------
        anvil_top_z    : Z coordinate of the anvil top face.
        particle_size  : nominal particle diameter (metres).
        init_gap_frac  : fraction of particle_size for the gap
                         (default 0.5 → half a particle).

        Returns
        -------
        float : world Z of the billet center after set-down.
        """
        lo, hi = self.aabb()
        half_height = (hi[2] - lo[2]) / 2.0
        gap = init_gap_frac * particle_size
        return anvil_top_z + half_height + gap

    def apply_rotation(
        self,
        angle_deg:   float,
        axis:        str,
        pivot_world: Optional[np.ndarray] = None
    ) -> None:
        """
        Rotate the particle position cloud about a world-space
        pivot point and update R_cumulative.

        This mutates self.pos in place so that the AABB of the
        rotated cloud can be read by remesh_scene() to size the
        new billet Box morph correctly.

        All coordinate metadata is explicit:
          - axis: "x" | "y" | "z"
          - pivot_world: (3,) array in world coords.
            If None, the billet centroid is used.

        Only positions are rotated here.  Velocities and F are
        NOT stored in BilletState (v5) and are not needed —
        the next remesh always starts fresh.

        Parameters
        ----------
        angle_deg   : degrees, right-hand rule about the axis.
        axis        : "x", "y", or "z".
        pivot_world : (3,) world-space pivot.  None → centroid.
        """
        # build rotation matrix
        theta = np.deg2rad(angle_deg)
        c, s  = np.cos(theta), np.sin(theta)

        if axis.lower() == "x":
            R = np.array([
                [1,  0,  0],
                [0,  c, -s],
                [0,  s,  c],
            ], dtype=np.float64)
        elif axis.lower() == "y":
            R = np.array([
                [ c,  0,  s],
                [ 0,  1,  0],
                [-s,  0,  c],
            ], dtype=np.float64)
        elif axis.lower() == "z":
            R = np.array([
                [c, -s,  0],
                [s,  c,  0],
                [0,  0,  1],
            ], dtype=np.float64)
        else:
            raise ValueError(f"axis must be 'x','y','z', got '{axis}'")

        # pivot
        if pivot_world is None:
            pivot_world = self.centroid()
        pivot_world = np.asarray(pivot_world, dtype=np.float64)

        # rotate positions
        p_local   = self.pos - pivot_world       # (N,3)
        p_rotated = (R @ p_local.T).T            # (N,3)
        self.pos  = p_rotated + pivot_world

        # accumulate rotation 
        self.R_cumulative = R @ self.R_cumulative
        self.rotation_idx += 1

    def translate_to_z(self, new_center_z: float) -> None:
        """
        Translate all particle positions so that the cloud
        centroid sits at new_center_z in the Z direction.
        X and Y centroids are re-centred to 0.
        """
        current_centroid = self.centroid()
        delta = np.array([
            -current_centroid[0],          # re-centre X to 0
            -current_centroid[1],          # re-centre Y to 0
            new_center_z - current_centroid[2],
        ], dtype=np.float64)
        self.pos += delta

    def apply_world_translation(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
    ) -> None:
        """
        Translate all particle positions by a fixed world-space
        offset vector (dx, dy, dz) in metres.

        This is always in WORLD coordinates — it does not depend
        on how the billet has been rotated.  Use this to slide
        the billet to a different position on the anvil before
        the next punch strike.

        Mutates self.pos in place.  R_cumulative is unchanged
        (a pure translation carries no rotational information).
        """
        self.pos += np.array([dx, dy, dz], dtype=np.float64)


# Stroke profile
class StrokeProfile:
    """
    Piecewise-constant velocity profile for a single punch
    strike (down then up).

    Constructed from a StrikeConfig; velocity(t) returns
    the signed Z-velocity of the punch at simulation time t.
    Negative Z → punch moves downward (toward billet).
    """

    def __init__(self, strike: StrikeConfig):
        v = strike.velocity_ms

        down_time = strike.down_dist_m / v
        up_time   = strike.up_dist_m   / v

        # list of (duration_s, signed_velocity_ms) segments
        self.segments = [
            (down_time, -v),   # move down
            (up_time,    v),   # retract up
        ]

    def velocity(self, t: float) -> float:
        """Signed Z-velocity (m/s) of punch at time t (s)."""
        elapsed = 0.0
        for duration, vel in self.segments:
            if elapsed <= t < elapsed + duration:
                return vel
            elapsed += duration
        return 0.0   # stroke complete

# Tool class 
class Tool:
    """
    Wraps a Genesis Tool entity and tracks its Z position.
    set_velocity + set_position are called every step (required
    by the Genesis MPM coupler — see TOOL DRIVE PATTERN above).
    """

    def __init__(self, entity, start_z: float):
        self.entity = entity
        self.z      = start_z

    def update(self, velocity: float, dt: float) -> None:
        """Advance Z by velocity*dt and push to Genesis."""
        self.z += velocity * dt

        self.entity.set_velocity(
            vel=np.array([[0.0, 0.0, velocity]])
        )
        self.entity.set_position(
            pos=np.array([[0.0, 0.0, self.z]])
        )

    def reset_z(self, new_z: float) -> None:
        """
        Teleport the tool to a new Z (used when the scene is
        rebuilt after a remesh and the punch must be placed at
        its correct position without a velocity step).
        """
        self.z = new_z
        self.entity.set_position(
            pos=np.array([[0.0, 0.0, self.z]])
        )
        self.entity.set_velocity(
            vel=np.array([[0.0, 0.0, 0.0]])
        )


#instantiate configurations 

cfg = ForgeConfig(
    billet_x=1.0 * INCH,
    billet_y=1.0 * INCH,
    billet_z=3.0 * INCH,

    anvil_x=4.0 * INCH,
    anvil_y=4.0 * INCH,
    anvil_thickness=0.5 * INCH,

    punch_x=2.0 * INCH,
    punch_y=2.0 * INCH,
    punch_thickness=0.5 * INCH,
)

solver_cfg = SolverConfig()

steel = MaterialConfig(
    E=20e9,
    nu=0.30,
    rho=7700.0,
)


###############################################################
# DERIVED GEOMETRY CONSTANTS
# (computed once here; carried through remeshes explicitly)
###############################################################

# Nominal particle size for the chosen grid density
# (half a grid cell in each linear dimension)
PARTICLE_SIZE = (1.0 / solver_cfg.grid_density) / 2.0   # ~5 mm

# Gap between billet bottom and anvil top on every set-down
INIT_GAP = PARTICLE_SIZE * 0.5                            # ~2.5 mm

# Z of the anvil centre (anvil is centred at table_z=0)
table_z = 0.0

# Z of the anvil TOP face
ANVIL_TOP_Z = table_z + cfg.anvil_thickness / 2.0

# Initial billet Z-centre (before any strike)
INITIAL_BILLET_CENTER_Z = (
    ANVIL_TOP_Z
    + cfg.billet_z / 2.0
    + INIT_GAP
)

# Initial punch Z-centre (1 mm clearance above billet top)
INITIAL_PUNCH_Z = (
    INITIAL_BILLET_CENTER_Z
    + cfg.billet_z / 2.0
    + cfg.punch_thickness / 2.0
    + 0.001          # 1 mm clearance
)

# Alias kept for rendering functions that reference initial_punch_z
initial_punch_z = INITIAL_PUNCH_Z

# Domain bounds
def compute_domain(cfg: ForgeConfig):
    """
    Compute MPM grid lower/upper bounds from forge geometry.
    Returns (lower_tuple, upper_tuple) each length 3.
    """
    half_width = max(
        cfg.anvil_x,
        cfg.anvil_y,
        cfg.billet_x * 3.0,
        cfg.billet_y * 3.0,
    ) / 2.0

    lower = (
        -half_width - cfg.lateral_margin,
        -half_width - cfg.lateral_margin,
        -cfg.anvil_thickness - cfg.vertical_margin,
    )
    upper = (
        half_width + cfg.lateral_margin,
        half_width + cfg.lateral_margin,
        cfg.anvil_thickness
        + cfg.billet_z
        + cfg.punch_thickness
        + cfg.vertical_margin,
    )
    return lower, upper


LOWER, UPPER = compute_domain(cfg)



# FAILURE THRESHOLDS
CFL_WARN           = 0.10
CFL_CRITICAL       = 1.00
J_INVERT_WARN      = 0.05
J_EXPLODE_WARN     = 2.50
STALL_BAND         = 0.001
STRAIN_JUMP        = 5.0
PVEL_WARN          = 15.0
PARTICLE_MASS_WARN = 1e-9


#################################################################
# OBJ GENERATOR
#################################################################
# gs.materials.Tool requires gs.morphs.Mesh (not Box).
# We generate watertight box OBJ files at the ORIGIN (0,0,0)
# and set the real-world position via set_position() every step.
# The OBJ is a unit-normalised template; actual geometry comes
# from the half-extents we pass in.
###################################################################
def write_box_obj(path: str, hx: float, hy: float, hz: float) -> None:
    """
    Write a centred axis-aligned box as a triangulated OBJ.
    Half-extents: hx, hy, hz  (box spans ±hx, ±hy, ±hz).
    12 triangles, proper outward normals, no UVs needed.
    """
    v = [
        (-hx, -hy, -hz),  # 1
        ( hx, -hy, -hz),  # 2
        ( hx,  hy, -hz),  # 3
        (-hx,  hy, -hz),  # 4
        (-hx, -hy,  hz),  # 5
        ( hx, -hy,  hz),  # 6
        ( hx,  hy,  hz),  # 7
        (-hx,  hy,  hz),  # 8
    ]
    # faces as 1-indexed vertex triples (two triangles per quad face)
    faces = [
        (1,3,2),(1,4,3),   # -Z
        (5,6,7),(5,7,8),   # +Z
        (1,2,6),(1,6,5),   # -Y
        (3,4,8),(3,8,7),   # +Y
        (1,5,8),(1,8,4),   # -X
        (2,3,7),(2,7,6),   # +X
    ]
    with open(path, "w") as fh:
        fh.write("# auto-generated box OBJ\n")
        for vx, vy, vz in v:
            fh.write(f"v {vx:.8f} {vy:.8f} {vz:.8f}\n")
        for a, b, c in faces:
            fh.write(f"f {a} {b} {c}\n")


# Write punch and anvil OBJ files (centred at origin)
PUNCH_OBJ = "output/meshes/punch.obj"
ANVIL_OBJ = "output/meshes/anvil.obj"

write_box_obj(
    PUNCH_OBJ,
    hx=cfg.punch_x / 2,
    hy=cfg.punch_y / 2,
    hz=cfg.punch_thickness / 2.0,
)
write_box_obj(
    ANVIL_OBJ,
    hx=cfg.anvil_x / 2,
    hy=cfg.anvil_y / 2,
    hz=cfg.anvil_thickness / 2.0,
)
print(f"OBJ meshes written → {PUNCH_OBJ}, {ANVIL_OBJ}")


# ##############################################################
# SCENE BUILDER
# ##############################################################
# Encapsulated so that remesh_scene() can call it identically
# to the initial build.  Returns a dict with all Genesis handles.
#
# billet_center    : (3,) world position for the billet Box centre.
# billet_aabb_size : (3,) full extents (x,y,z) of the billet Box.
# punch_z          : world Z centre of the punch at build time.
#
# NOTE: gs_init_fresh() is called here (not in remesh_scene)
#   so that the build/remesh path is symmetric.  build_scene()
#   always starts from a clean Genesis runtime.
#################################################################
def build_scene(
    billet_center: np.ndarray,
    billet_aabb_size: np.ndarray,
    punch_z: float,
) -> dict:
    """
    Destroy any existing Genesis runtime, initialise a new one,
    create and build a complete scene.

    Parameters
    ----------
    billet_center    : (3,) world-space centre for billet Box morph.
    billet_aabb_size : (3,) full extents (x,y,z) of the billet Box.
    punch_z          : world Z centre of the punch Tool mesh.

    Returns
    -------
    dict with keys: scene, billet, punch, table, camera, cell_size
    """
    # destroy old runtime if present, start fresh 
    gs_init_fresh()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=solver_cfg.dt,
            substeps=solver_cfg.substeps,
        ),
        mpm_options=gs.options.MPMOptions(
            grid_density=solver_cfg.grid_density,
            enable_CPIC=solver_cfg.enable_cpic,
            lower_bound=LOWER,
            upper_bound=UPPER,
        ),
    )

    # Avil  (Tool + Mesh, stationary) 
    table = scene.add_entity(
        morph=gs.morphs.Mesh(
            file=ANVIL_OBJ,
            pos=(0.0, 0.0, table_z),
            euler=(0.0, 0.0, 0.0),
            scale=1.0,
        ),
        material=gs.materials.Tool(friction=0.3),
    )

    # BILLET  (MPM ElastoPlastic)
    # The Box is sized to the current billet AABB so Genesis
    # seeds particles on a uniform grid inside that volume.
    # Particles start fresh: zero velocity, identity F.
    billet = scene.add_entity(
        morph=gs.morphs.Box(
            size=(
                float(billet_aabb_size[0]),
                float(billet_aabb_size[1]),
                float(billet_aabb_size[2]),
            ),
            pos=(
                float(billet_center[0]),
                float(billet_center[1]),
                float(billet_center[2]),
            ),
        ),
        material=gs.materials.MPM.ElastoPlastic(
            E=steel.E,
            nu=steel.nu,
            rho=steel.rho,
        ),
    )

    punch = scene.add_entity(
        morph=gs.morphs.Mesh(
            file=PUNCH_OBJ,
            pos=(0.0, 0.0, punch_z),
            euler=(0.0, 0.0, 0.0),
            scale=1.0,
        ),
        material=gs.materials.Tool(friction=0.3),
    )

    camera = scene.add_camera(
        res=(1280, 720),
        pos=(0.18, -0.22, 0.12),
        lookat=(0.0, 0.0, 0.05),
    )

    scene.build()

    # Authoritative cell_size; computed ONCE post build
    # using the domain width and grid density (BUG-5 fix).
    domain_width = UPPER[0] - LOWER[0]
    cell_size    = domain_width / float(solver_cfg.grid_density)

    # CFL check
    c_wave = np.sqrt(steel.E / steel.rho)
    cfl    = c_wave * solver_cfg.dt / cell_size
    assert cfl < 0.5, (
        f"CFL={cfl:.3f} violates stability — reduce dt or E"
    )

    return dict(
        scene=scene,
        billet=billet,
        punch=punch,
        table=table,
        camera=camera,
        cell_size=cell_size,
    )


# ###########################################################
# REMESH FUNCTION  (v5 — geometry-only, no state injection)
# ###########################################################
# Destroys the current Genesis scene and rebuilds a brand-new
# simulation whose billet Box morph is sized to match the
# DEFORMED GEOMETRY captured in billet_state.pos.
# 
# No particle state variables are stored, the geometry of the billet is preserved
# Upon remesh new particles are created like if you were restarting the simulation
#
# The purpose of this is to eliminate accumulated numerical error in the MPM
# particle state.
#
# The punch is placed at punch_z_world after rebuild.
# The table_tool and punch_tool Tool wrappers are replaced.
###############################################################

def remesh_scene(
    billet_state:  "BilletState",
    punch_z_world: float,
) -> dict:
    """
    Tear down the Genesis scene and rebuild it with a fresh
    billet sized to the AABB of billet_state.pos.

    Particles start at rest with identity deformation gradients.
    No state injection occurs 

    Parameters
    ----------
    billet_state   : BilletState whose .pos gives the current
                     AABB (deformed geometry).  vel and F are
                     not used — they no longer exist in v5.
    punch_z_world  : Z centre of punch in the rebuilt scene.

    Returns
    -------
    dict with same keys as build_scene() plus:
        punch_tool  : Tool wrapper around new punch entity
        table_tool  : Tool wrapper around new table entity
    """
    # compute morph parameters from current AABB 
    lo, hi        = billet_state.aabb()
    aabb_size     = hi - lo                          # (3,)
    billet_center = (lo + hi) / 2.0                 # (3,)

    # Guard in place for if billet has collapsed to near zero or zero thickness
    # In any dimension, pad to at least 2x particle size 
    # this gives Genesis a non degenerate volume to fill.
    min_dim = 2.0 * PARTICLE_SIZE
    aabb_size = np.maximum(aabb_size, min_dim)

    #rebuild scene (destroys old runtime inside)
    handles = build_scene(
        billet_center=billet_center,
        billet_aabb_size=aabb_size,
        punch_z=punch_z_world,
    )
    scene  = handles["scene"]
    punch  = handles["punch"]
    table  = handles["table"]

    #rebuild Tool wrappers
    punch_tool = Tool(punch, punch_z_world)
    table_tool = Tool(table, table_z)

    punch_tool.reset_z(punch_z_world)
    table_tool.reset_z(table_z)

    handles["punch_tool"] = punch_tool
    handles["table_tool"] = table_tool

    return handles



# Capture billet geometry from Genesis 
def capture_billet_geometry(
    billet_entity,
    existing_state: Optional["BilletState"] = None,
) -> "BilletState":
    """
    Read current particle positions from a Genesis billet
    entity and package into a BilletState

    If existing_state is provided, its R_cumulative and
    rotation_idx are preserved

    Parameters
    ----------
    billet_entity  : Genesis MPM entity.
    existing_state : optional prior BilletState for metadata.

    Returns
    -------
    BilletState with .pos populated.
    """
    raw = billet_entity.get_state()

    pos = np.asarray(raw.pos.cpu().numpy())

    # squeeze leading batch dimensions
    while pos.ndim > 2:
        pos = pos[0]

    R_cum   = existing_state.R_cumulative if existing_state else np.eye(3)
    rot_idx = existing_state.rotation_idx if existing_state else 0

    return BilletState(
        pos=pos.copy(),
        R_cumulative=R_cum.copy(),
        rotation_idx=rot_idx,
    )

# ##############################################################
# ROTATION VIA MESH CONVERSION
# ##############################################################
# Process: 
#   1. Voxelise the particle cloud into a density grid
#   2. Run marching cubes to extract a surface mesh
#   3. Rotate the mesh vertices
#   4. Compute the AABB of the rotated mesh
#   5. Pass the rotated AABB to remesh_scene() so Genesis
#      seeds fresh particles inside the rotated volume
# ##############################################################

def rotate_billet_via_mesh(
    billet_state:  "BilletState",
    angle_deg:     float,
    axis:          str,
    pivot_world:   Optional[np.ndarray] = None,
    voxel_res:     int = 32,
) -> "BilletState":
    """
    Convert particle cloud to surface mesh, rotate the mesh,
    then compute a new BilletState whose .pos contains points
    sampled from the rotated mesh volume (for AABB purposes).

    Parameters
    ----------
    billet_state : current BilletState with .pos (particle positions)
    angle_deg    : rotation angle in degrees (right-hand rule)
    axis         : "x", "y", or "z"
    pivot_world  : (3,) world-space pivot. None → billet centroid
    voxel_res    : resolution of voxel grid for marching cubes
                   (higher = more faithful surface, slower)

    Returns
    -------
    New BilletState with:
      .pos           – vertices of rotated mesh surface (used for AABB)
      .R_cumulative  – updated cumulative rotation
      .rotation_idx  – incremented
    """

    pos = billet_state.pos  # (N, 3)

    # Step 1: build rotation matrix 
    theta = np.deg2rad(angle_deg)
    c, s  = np.cos(theta), np.sin(theta)

    if axis.lower() == "x":
        R = np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=np.float64)
    elif axis.lower() == "y":
        R = np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=np.float64)
    elif axis.lower() == "z":
        R = np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=np.float64)
    else:
        raise ValueError(f"axis must be 'x','y','z', got '{axis}'")

    # Step 2: voxelise particle cloud
    lo = pos.min(axis=0)
    hi = pos.max(axis=0)
    pad = (hi - lo) * 0.05  # 5% padding so surface closes at edges
    lo -= pad
    hi += pad

    # build voxel grid
    xs = np.linspace(lo[0], hi[0], voxel_res + 1)
    ys = np.linspace(lo[1], hi[1], voxel_res + 1)
    zs = np.linspace(lo[2], hi[2], voxel_res + 1)

    # bin particles into voxels
    xi = np.clip(
        ((pos[:,0] - lo[0]) / (hi[0] - lo[0]) * voxel_res).astype(int),
        0, voxel_res - 1,
    )
    yi = np.clip(
        ((pos[:,1] - lo[1]) / (hi[1] - lo[1]) * voxel_res).astype(int),
        0, voxel_res - 1,
    )
    zi = np.clip(
        ((pos[:,2] - lo[2]) / (hi[2] - lo[2]) * voxel_res).astype(int),
        0, voxel_res - 1,
    )

    density = np.zeros((voxel_res, voxel_res, voxel_res), dtype=np.float32)
    np.add.at(density, (xi, yi, zi), 1.0)

    # gaussian smooth so marching cubes gets a clean isosurface

    density = gaussian_filter(density, sigma=1.2)

    # Step 3: marching cubes → surface mesh 
    iso_level = density.max() * 0.15   # 15% of peak density
    try:
        verts_vox, faces, normals, _ = marching_cubes(
            density, level=iso_level, spacing=(1.0, 1.0, 1.0),
        )
    except (ValueError, RuntimeError) as e:
        print(f"  [rotate_via_mesh] marching_cubes failed ({e}), "
              f"falling back to point-cloud rotation")
        # fallback: just rotate the point cloud directly
        billet_state.apply_rotation(angle_deg, axis, pivot_world)
        return billet_state

    # convert voxel indices back to world coordinates
    vox_spacing = (hi - lo) / voxel_res
    verts_world = verts_vox * vox_spacing[np.newaxis, :] + lo  # (V, 3)

    print(f"  [rotate_via_mesh] surface mesh: "
          f"{len(verts_world)} verts, {len(faces)} faces")

    # Step 4: rotate mesh vertices about pivot 
    if pivot_world is None:
        pivot_world = billet_state.centroid()
    pivot_world = np.asarray(pivot_world, dtype=np.float64)

    verts_local   = verts_world - pivot_world          # (V, 3)
    verts_rotated = (R @ verts_local.T).T + pivot_world  # (V, 3)

    # Step 5: also rotate the original particle positions
    #    returns correct centroid for set down Z calculation
    pos_local   = pos - pivot_world
    pos_rotated = (R @ pos_local.T).T + pivot_world

    # Use mesh vertices as the "positions" for AABB calculation
    combined_pts = np.vstack([verts_rotated, pos_rotated])

    # Step 6: build new BilletState 
    new_state = BilletState(
        pos=combined_pts,
        R_cumulative=(R @ billet_state.R_cumulative).copy(),
        rotation_idx=billet_state.rotation_idx + 1,
    )

    print(f"  [rotate_via_mesh] rotated AABB: "
          f"X=[{combined_pts[:,0].min()*1e3:.1f}, {combined_pts[:,0].max()*1e3:.1f}] mm  "
          f"Y=[{combined_pts[:,1].min()*1e3:.1f}, {combined_pts[:,1].max()*1e3:.1f}] mm  "
          f"Z=[{combined_pts[:,2].min()*1e3:.1f}, {combined_pts[:,2].max()*1e3:.1f}] mm")

    return new_state

# ############################################################
# TOOL PATH DEFINITION
##############################################################
# Edit this list to prescribe the full forging sequence.
# Each ToolPathStep is executed in order.
#
# The example below performs:
#   1. First punch strike (with 1 intra-strike remesh)
#   2. Rotate 90° about X axis
#   3. Second punch strike (with 1 intra-strike remesh)
# #############################################################

TOOL_PATH: List[ToolPathStep] = [
    ToolPathStep(
        kind="punch",
        strike_cfg=StrikeConfig(
            down_dist_m=0.0381,
            up_dist_m=0.0381,
            velocity_ms=0.5,
            n_steps=None,
            punch_remesh_times=1,
        ),
    ),
    ToolPathStep(
        kind="translate",
        translate_x=1.0 * INCH,   # slide 1 inch along world X
        translate_y=0.0,
        translate_z=0.0,
    ),
    ToolPathStep(
        kind="punch",
        strike_cfg=StrikeConfig(
            down_dist_m=0.0381,
            up_dist_m=0.0381,
            velocity_ms=0.5,
            n_steps=None,
            punch_remesh_times=1,
        ),
    ),
]

# Fill in default n_steps from SolverConfig where not overridden
for _step in TOOL_PATH:
    if _step.kind == "punch" and _step.strike_cfg.n_steps is None:
        _step.strike_cfg.n_steps = solver_cfg.n_steps



###########################################################
# BEGIN helper functions
###########################################################

def squeeze2(arr, ndim):
    """Squeeze leading size-1 dimensions down to ndim."""
    a = np.asarray(arr)
    while a.ndim > ndim:
        if a.shape[0] != 1:
            break
        a = a[0]
    return a


def get_pos(st): return squeeze2(st.pos.cpu().numpy(), 2)
def get_F(st):   return squeeze2(st.F.cpu().numpy(),   3)
def get_vel(st): return squeeze2(st.vel.cpu().numpy(), 2)


def jacobians(F: np.ndarray) -> np.ndarray:
    """Per-particle volume Jacobian J = det(F), shape (N,)."""
    return np.linalg.det(F)


def equiv_strain(F: np.ndarray) -> np.ndarray:
    """
    Green-Lagrange equivalent strain per particle.
    E = 0.5*(F^T F - I);  ε_eq = ||E||_F
    """
    C = np.einsum("nij,nik->njk", F, F)
    E = 0.5 * (C - np.eye(3))
    return np.sqrt(np.einsum("nij,nij->n", E, E))


def compute_cfl(
    vel:    np.ndarray,
    dt:     float,
    dx:     float,
) -> Tuple[float, np.ndarray]:
    """
    Return (cfl_max, speed_per_particle) given particle
    velocities, time step dt, and cell size dx.
    """
    spd = np.linalg.norm(vel, axis=-1)
    return float(spd.max()) * dt / dx, spd


def pmass(solver) -> Tuple[float, float]:
    """
    Return (min_mass, mean_mass) from the MPM solver's
    particle_info.  Returns (nan, nan) on any failure.
    """
    try:
        arr = np.asarray(
            solver.particles_info.mass.cpu().numpy()
        ).ravel()
        arr = arr[arr > 0]
        if arr.size:
            return float(arr.min()), float(arr.mean())
    except Exception:
        pass
    return np.nan, np.nan


def fig_bgr(fig, W: int, H: int) -> np.ndarray:
    """
    Convert a Matplotlib figure to a BGR uint8 array of
    size (H, W, 3), resizing if necessary.
    """
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    buf  = np.frombuffer(
        fig.canvas.buffer_rgba(), np.uint8
    ).reshape(h, w, 4)
    bgr  = cv2.cvtColor(buf[:, :, :3], cv2.COLOR_RGB2BGR)
    if bgr.shape[1] != W or bgr.shape[0] != H:
        bgr = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
    return bgr


def box_poly(
    ax,
    cx, cy, cz,
    dx, dy, dz,
    col,
    alpha=0.15,
) -> None:
    """
    Add a transparent wireframe box to a 3D Matplotlib axes.
    Centre (cx,cy,cz), full extents (dx,dy,dz).
    """
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
    ax.add_collection3d(
        Poly3DCollection(
            verts,
            alpha=alpha,
            linewidths=0.5,
            edgecolors=col,
            facecolors=col,
        )
    )




   
def draw_local_axes(
    ax,
    center_mm,
    R_cumulative,
    axis_length_mm=35.0,
    linewidth=5.0,
):
    """
    Draw billet-local coordinate system.

    X+ : Ferrari red
    Y+ : mustard yellow
    Z+ : dark blue

    Axes originate at billet centroid and extend only
    in the positive local directions.
    """

    origin = np.asarray(center_mm)

    x_axis = R_cumulative @ np.array([1.0, 0.0, 0.0])
    y_axis = R_cumulative @ np.array([0.0, 1.0, 0.0])
    z_axis = R_cumulative @ np.array([0.0, 0.0, 1.0])

    colors = {
        "x": "#D40000",  # Ferrari-style red
        "y": "#D8A300",  # mustard yellow
        "z": "#003366",  # dark blue
    }

    axes = [
        (x_axis, colors["x"]),
        (y_axis, colors["y"]),
        (z_axis, colors["z"]),
    ]

    for direction, color in axes:

        end = origin + direction * axis_length_mm

        ax.plot(
            [origin[0], end[0]],
            [origin[1], end[1]],
            [origin[2], end[2]],
            color=color,
            linewidth=linewidth,
            solid_capstyle="butt",
            zorder=1000,
        )

def draw_local_axes_xz(
    ax,
    center_mm,
    R_cumulative,
    axis_length_mm,
):
    origin_x = center_mm[0]
    origin_z = center_mm[2]

    axes = [
        (R_cumulative @ np.array([1,0,0]), "#D40000", "X"),
        (R_cumulative @ np.array([0,1,0]), "#D8A300", "Y"),
        (R_cumulative @ np.array([0,0,1]), "#003366", "Z"),
    ]

    for direction, color, label in axes:

        end_x = origin_x + direction[0] * axis_length_mm
        end_z = origin_z + direction[2] * axis_length_mm

        ax.plot(
            [origin_x, end_x],
            [origin_z, end_z],
            color=color,
            linewidth=5,
            solid_capstyle="butt",
        )

        ax.text(
            end_x,
            end_z,
            label,
            color=color,
            fontweight="bold",
        )

def draw_local_axes_xy(
    ax,
    center_mm,
    R_cumulative,
    axis_length_mm,
):
    origin_x = center_mm[0]
    origin_y = center_mm[1]

    axes = [
        (R_cumulative @ np.array([1,0,0]), "#D40000", "X"),
        (R_cumulative @ np.array([0,1,0]), "#D8A300", "Y"),
        (R_cumulative @ np.array([0,0,1]), "#003366", "Z"),
    ]

    for direction, color, label in axes:

        end_x = origin_x + direction[0] * axis_length_mm
        end_y = origin_y + direction[1] * axis_length_mm

        ax.plot(
            [origin_x, end_x],
            [origin_y, end_y],
            color=color,
            linewidth=5,
            solid_capstyle="butt",
        )

        ax.text(
            end_x,
            end_y,
            label,
            color=color,
            fontweight="bold",
        )

###########################################################
# END helper functions
###########################################################

# Strain colormap 
STRAIN_CMAP = mcolors.LinearSegmentedColormap.from_list(
    "forge_strain",
    [
        (0.00, "#ffffff"),  # white
        (0.10, "#ffff00"),  # yellow
        (0.35, "#F6AA06"),  # orange
        (0.70, "#ff0000"),  # red
        (1.00, "#8000ff"),  # purple
    ],
)

ELASTIC_LIMIT  = 0.02
MODERATE_FLOW  = 0.10
PLASTIC_FLOW   = 0.25
SEVERE_FLOW    = 0.60

STRAIN_VMIN = 0.0
STRAIN_VMAX = 0.60

STRAIN_NORM = mcolors.Normalize(vmin=0.0, vmax=SEVERE_FLOW)




#####################################################
# BEGIN solid deformation rendering functions 
#####################################################
def render_solid_3d(pos, eps, J, step, t, pz, flags, frame_label="", R_cumulative=np.eye(3)):
    pos_mm     = pos * 1000.0
    disp_mm    = (initial_punch_z - pz) * 1000.0
    billet_h_mm = (pos[:,2].max() - pos[:,2].min()) * 1000.0
    billet_w_mm = (pos[:,0].max() - pos[:,0].min()) * 1000.0

    fig = plt.figure(figsize=(16,10), dpi=80)
    fig.patch.set_facecolor("#0a0a0a")
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#0a0a0a")

    bbox = pos_mm.max(axis=0) - pos_mm.min(axis=0)
    axis_length_mm = 0.65 * np.max(bbox)    

    centroid_mm = pos_mm.mean(axis=0)

    draw_local_axes(
    ax,
    centroid_mm,
    R_cumulative,
    axis_length_mm=axis_length_mm,
    linewidth=6,
    )

    try:
        if len(pos_mm) >= 4:
            hull  = ConvexHull(pos_mm)
            cmap  = STRAIN_CMAP
            norm  = STRAIN_NORM
            faces = []
            cols  = []
            for s in hull.simplices:
                faces.append([tuple(pos_mm[i]) for i in s])
                cols.append(cmap(norm(float(eps[s].mean()))))
            ax.add_collection3d(
                Poly3DCollection(
                    faces,
                    facecolors=cols,
                    alpha=0.88,
                    edgecolors='k',
                    linewidths=2
                )
            )

            # punch
            box_poly(ax, 0, 0, pz*1000,
                     cfg.punch_x*1000, cfg.punch_y*1000,
                     cfg.punch_thickness*1000, "cyan", 0.15)
            # anvil
            box_poly(ax, 0, 0, table_z*1000,
                     cfg.anvil_x*1000, cfg.anvil_y*1000,
                     cfg.anvil_thickness*1000, "cyan", 0.15)
            
    
            
    except Exception:
        ax.scatter(
            pos_mm[:,0], pos_mm[:,1], pos_mm[:,2],
            c=eps, cmap=STRAIN_CMAP, s=1.5, norm=STRAIN_NORM,
        )

    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_zlim(-10, 130)
    ax.view_init(elev=25, azim=-50)
    ax.set_title("3D Forged Billet Geometry (Equivalent Strain Colored)",
                 color="white", fontsize=12, fontweight="bold")
    ax.set_xlabel("X Position [mm]", color="white")
    ax.set_ylabel("Y Position [mm]", color="white")
    ax.set_zlabel("Height Z [mm]", color="white")
    ax.tick_params(colors="white")

    cb = fig.colorbar(
        plt.cm.ScalarMappable(norm=STRAIN_NORM, cmap=STRAIN_CMAP),
        ax=ax, shrink=0.7,
    )
    cb.set_label("Equivalent Strain ε_eq [-]", color="white")
    plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color="white")

    fig.text(0.02, 0.96,
             f"Punch Displacement = {disp_mm:.2f} mm\n"
             f"Billet Height = {billet_h_mm:.2f} mm\n"
             f"Billet Width  = {billet_w_mm:.2f} mm\n"
             f"Particles     = {pos.shape[0]}",
             color="white", fontsize=10)

    if frame_label:
        fig.text(0.98, 0.96, frame_label, color="#ffcc00",
                 fontsize=11, fontweight="bold", ha="right")

    fig.suptitle(
        f"FORGING SIMULATION | {frame_label} | STEP {step:05d} | t={t:.4f}s",
        color="white", fontsize=14, fontweight="bold",
    )
    bgr = fig_bgr(fig, SOLID_W, SOLID_H)
    plt.close(fig)
    return bgr


def render_solid_xz(pos, eps, J, step, t, pz, flags, frame_label="", R_cumulative=np.eye(3)):
    pos_mm      = pos * 1000.0
    disp_mm     = (initial_punch_z - pz) * 1000.0
    billet_h_mm = (pos[:,2].max() - pos[:,2].min()) * 1000.0
    billet_w_mm = (pos[:,0].max() - pos[:,0].min()) * 1000.0

    fig, ax = plt.subplots(figsize=(16,10), dpi=80)
    fig.patch.set_facecolor("#0a0a0a")
    ax.set_facecolor("#111111")

    centroid_mm = pos_mm.mean(axis=0)
    bbox = pos_mm.max(axis=0) - pos_mm.min(axis=0)
    axis_length_mm = 0.65 * np.max(bbox)    

    draw_local_axes_xz(
        ax,
        centroid_mm,
        R_cumulative,
        axis_length_mm,
    )

    sc = ax.scatter(
        pos_mm[:,0], pos_mm[:,2],
        c=eps, cmap=STRAIN_CMAP, s=3, alpha=0.85,
        linewidths=0, norm=STRAIN_NORM,
    )

    # punch outline
    punch_x_mm = cfg.punch_x * 1000
    punch_z_mm = cfg.punch_thickness * 1000
    ax.add_patch(MPLRect(
        (-punch_x_mm/2, pz*1000 - punch_z_mm/2),
        punch_x_mm, punch_z_mm,
        facecolor="cyan", edgecolor="white", linewidth=2, alpha=0.15,
    ))

    # anvil outline
    anvil_x_mm = cfg.anvil_x * 1000
    anvil_z_mm = cfg.anvil_thickness * 1000
    ax.add_patch(MPLRect(
        (-anvil_x_mm/2, table_z*1000 - anvil_z_mm/2),
        anvil_x_mm, anvil_z_mm,
        facecolor="cyan", edgecolor="white", linewidth=2, alpha=0.2,
    ))

    ax.set_xlim(-50, 50)
    ax.set_ylim(-10, 130)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.set_xticks(np.arange(-50, 51, 10))
    ax.set_yticks(np.arange(-10, 131, 10))
    ax.set_title("Side View (X-Z Cross Section)",
                 color="white", fontsize=14, fontweight="bold")
    ax.set_xlabel("Width X [mm]", color="white")
    ax.set_ylabel("Height Z [mm]", color="white")
    ax.tick_params(colors="white")
    cb = fig.colorbar(sc)
    cb.set_label("Equivalent Strain ε_eq [-]", color="white")
    plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color="white")

    fig.text(0.02, 0.96,
             f"Punch Displacement = {disp_mm:.2f} mm\n"
             f"Billet Height = {billet_h_mm:.2f} mm\n"
             f"Billet Width  = {billet_w_mm:.2f} mm\n"
             f"Maximum Strain = {eps.max():.4f}",
             color="white", fontsize=10)

    if frame_label:
        fig.text(0.98, 0.96, frame_label, color="#ffcc00",
                 fontsize=11, fontweight="bold", ha="right")

    fig.suptitle(
        f"FORGING SIDE PROFILE | {frame_label} | STEP {step:05d} | t={t:.4f}s",
        color="white", fontsize=14, fontweight="bold",
    )
    bgr = fig_bgr(fig, SOLID_W, SOLID_H)
    plt.close(fig)
    return bgr


def render_solid_xy(pos, eps, J, step, t, pz, flags, frame_label="", R_cumulative=np.eye(3)):
    pos_mm      = pos * 1000.0
    disp_mm     = (initial_punch_z - pz) * 1000.0
    billet_h_mm = (pos[:,2].max() - pos[:,2].min()) * 1000.0
    billet_w_mm = (pos[:,0].max() - pos[:,0].min()) * 1000.0

    fig, ax = plt.subplots(figsize=(16,10), dpi=80)
    fig.patch.set_facecolor("#0a0a0a")
    ax.set_facecolor("#111111")


    centroid_mm = pos_mm.mean(axis=0)
    bbox = pos_mm.max(axis=0) - pos_mm.min(axis=0)
    axis_length_mm = 0.65 * np.max(bbox)    

    draw_local_axes_xy(
        ax,
        centroid_mm,
        R_cumulative,
        axis_length_mm,
    )

    sc = ax.scatter(
        pos_mm[:,0], pos_mm[:,1],
        c=eps, cmap=STRAIN_CMAP, s=3, alpha=0.85,
        linewidths=0, norm=STRAIN_NORM,
    )

    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.set_xticks(np.arange(-50, 51, 10))
    ax.set_yticks(np.arange(-50, 51, 10))
    ax.set_title("Top View (X-Y Footprint Spread)",
                 color="white", fontsize=14, fontweight="bold")
    ax.set_xlabel("Width X [mm]", color="white")
    ax.set_ylabel("Width Y [mm]", color="white")
    ax.tick_params(colors="white")
    cb = fig.colorbar(sc)
    cb.set_label("Equivalent Strain ε_eq [-]", color="white")
    plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color="white")

    fig.text(0.02, 0.96,
             f"Punch Displacement = {disp_mm:.2f} mm\n"
             f"Billet Height = {billet_h_mm:.2f} mm\n"
             f"Billet Width  = {billet_w_mm:.2f} mm\n"
             f"Maximum Strain = {eps.max():.4f}",
             color="white", fontsize=10)

    if frame_label:
        fig.text(0.98, 0.96, frame_label, color="#ffcc00",
                 fontsize=11, fontweight="bold", ha="right")

    fig.suptitle(
        f"FORGING TOP PROFILE | {frame_label} | STEP {step:05d} | t={t:.4f}s",
        color="white", fontsize=14, fontweight="bold",
    )
    bgr = fig_bgr(fig, SOLID_W, SOLID_H)
    plt.close(fig)
    return bgr

#####################################################
# END solid deformation rendering functions 
#####################################################


# Render particle scattering
def render_particles(pos, eps, J, step, t, pz, flags, frame_label=""):
    fig = plt.figure(figsize=(18, 11.25), dpi=80)
    fig.patch.set_facecolor("#0d0d0d")
    gsl = fig.add_gridspec(
        2, 2, height_ratios=[8, 1.2],
        hspace=0.18, wspace=0.08,
        left=0.04, right=0.96, top=0.93, bottom=0.06,
    )
    ax1 = fig.add_subplot(gsl[0,0], projection="3d")
    ax2 = fig.add_subplot(gsl[0,1], projection="3d")
    aL1 = fig.add_subplot(gsl[1,0])
    aL2 = fig.add_subplot(gsl[1,1])

    tc = -cfg.anvil_thickness / 2.0
    XL, YL, ZL = (-0.05, 0.05), (-0.05, 0.05), (-0.01, 0.13)

    # Strain panel
    ax1.set_facecolor("#0d0d0d")
    e1d = eps.ravel()
    sc1 = ax1.scatter(
        pos[:,0], pos[:,1], pos[:,2],
        c=e1d, cmap="turbo",
        vmin=0, vmax=max(float(e1d.max()), 1e-6),
        s=2, alpha=0.92, linewidths=0, depthshade=False,
    )
    box_poly(ax1, 0,0,pz,   2*INCH, 2*INCH, cfg.punch_thickness, "#aaaaaa", 0.12)
    box_poly(ax1, 0,0,tc,   4*INCH, 4*INCH, cfg.anvil_thickness,  "#888888", 0.12)
    ax1.set_xlim(*XL); ax1.set_ylim(*YL); ax1.set_zlim(*ZL)
    ax1.view_init(22, -42)
    ax1.set_title("Equivalent Green-Lagrange Strain  ε_eq",
                  color="white", fontsize=12, pad=8, fontweight="bold")
    for lbl, s in [("X (m)", ax1.set_xlabel),
                   ("Y (m)", ax1.set_ylabel),
                   ("Z (m)", ax1.set_zlabel)]:
        s(lbl, color="#aaaaaa", fontsize=7, labelpad=4)
    ax1.tick_params(colors="#777777", labelsize=6)
    for p in (ax1.xaxis.pane, ax1.yaxis.pane, ax1.zaxis.pane):
        p.fill = False; p.set_edgecolor("#222222")
    cb1 = fig.colorbar(sc1, ax=ax1, shrink=0.6, pad=0.10, aspect=22)
    cb1.set_label("ε_eq [–]", color="white", fontsize=9)
    plt.setp(plt.getp(cb1.ax.axes, "yticklabels"), color="white")

    # Jacobian panel
    ax2.set_facecolor("#0d0d0d")
    J1d  = J.ravel()
    Jclp = np.clip(J1d, 0, 2)
    sc2  = ax2.scatter(
        pos[:,0], pos[:,1], pos[:,2],
        c=Jclp, cmap="RdYlGn",
        norm=mcolors.TwoSlopeNorm(vmin=0, vcenter=1, vmax=2),
        s=2, alpha=0.92, linewidths=0, depthshade=False,
    )
    box_poly(ax2, 0,0,pz,   2*INCH, 2*INCH, cfg.punch_thickness, "#aaaaaa", 0.12)
    box_poly(ax2, 0,0,tc,   4*INCH, 4*INCH, cfg.anvil_thickness,  "#888888", 0.12)
    ax2.set_xlim(*XL); ax2.set_ylim(*YL); ax2.set_zlim(*ZL)
    ax2.view_init(22, -42)
    ax2.set_title(
        "Volume Jacobian  J=det(F)  (red=compressed · green=dilated)",
        color="white", fontsize=12, pad=8, fontweight="bold",
    )
    for lbl, s in [("X (m)", ax2.set_xlabel),
                   ("Y (m)", ax2.set_ylabel),
                   ("Z (m)", ax2.set_zlabel)]:
        s(lbl, color="#aaaaaa", fontsize=7, labelpad=4)
    ax2.tick_params(colors="#777777", labelsize=6)
    for p in (ax2.xaxis.pane, ax2.yaxis.pane, ax2.zaxis.pane):
        p.fill = False; p.set_edgecolor("#222222")
    cb2 = fig.colorbar(sc2, ax=ax2, shrink=0.6, pad=0.10, aspect=22)
    cb2.set_label("J=det(F) [–]", color="white", fontsize=9)
    plt.setp(plt.getp(cb2.ax.axes, "yticklabels"), color="white")

    # Legend strips
    for ax_l, title, lines in [
        (aL1, "STRAIN COLOUR GUIDE", [
            (0.62, "Blue → ε≈0 (undeformed)",        "#55aaff"),
            (0.44, "Yellow/Green → moderate deformation", "#aadd55"),
            (0.26, "Red → high strain (contact zone)", "#ff4444"),
            (0.08, "ε_eq = ‖0.5(FᵀF−I)‖_F",          "#888888"),
        ]),
        (aL2, "JACOBIAN COLOUR GUIDE", [
            (0.62, "Red → J<1 (volume COMPRESSED)",     "#ff4444"),
            (0.44, "Yellow → J≈1 (reference)",          "#ddcc22"),
            (0.26, "Green → J>1 (DILATED, lateral spread)", "#44cc44"),
            (0.08, "J≤0=INVERTED(fatal). J>2.5=explosive.", "#888888"),
        ]),
    ]:
        ax_l.set_facecolor("#111111")
        ax_l.axis("off")
        ax_l.text(0.01, 0.85, title, color="white", fontsize=10,
                  fontweight="bold", transform=ax_l.transAxes)
        for y, txt, col in lines:
            ax_l.text(0.01, y, txt, color=col, fontsize=9,
                      transform=ax_l.transAxes,
                      style="italic" if y == 0.08 else "normal")

    disp_mm = (initial_punch_z - pz) * 1000
    fig.suptitle(
        f"HOT STEEL FORGING  ·  {frame_label}  ·  step {step:05d}"
        f"  ·  t={t:.4f}s  ·  punch↓{disp_mm:.1f}mm",
        color="white", fontsize=13, fontweight="bold", y=0.97,
    )

    wc = ("#ff4444" if ("INVERT" in flags or "CRITICAL" in flags)
          else "#ffaa00" if flags != "–" else "#55dd55")
    fig.text(
        0.5, 0.02,
        f"N={pos.shape[0]}  |  max_ε={float(e1d.max()):.4f}  |  "
        f"J_min={float(J1d.min()):.4f}  J_max={float(J1d.max()):.4f}"
        f"  |  FLAGS:{flags}",
        ha="center", va="bottom", color=wc, fontsize=9,
        bbox=dict(facecolor="#1a1a1a", edgecolor="#333333",
                  boxstyle="round,pad=0.4"),
        fontfamily="monospace",
    )

    bgr = fig_bgr(fig, PARTICLE_W, PARTICLE_H)
    plt.close(fig)
    return bgr


# Video writer dimensions

CAMERA_W,   CAMERA_H   = 1280, 720
PARTICLE_W, PARTICLE_H = 1440, 900
SOLID_W,    SOLID_H    = 1280, 800
FPS = 30


# initialize video writers 

camera_video   = cv2.VideoWriter(
    "output/forge_camera.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"), FPS, (CAMERA_W, CAMERA_H),
)
particle_video = cv2.VideoWriter(
    "output/forge_particles.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"), FPS, (PARTICLE_W, PARTICLE_H),
)
solid_3d_video = cv2.VideoWriter(
    "output/forge_solid_3d.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"), FPS, (SOLID_W, SOLID_H),
)
solid_xz_video = cv2.VideoWriter(
    "output/forge_solid_xz.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"), FPS, (SOLID_W, SOLID_H),
)
solid_xy_video = cv2.VideoWriter(
    "output/forge_solid_xy.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"), FPS, (SOLID_W, SOLID_H),
)

# CSV writer 
csv_file   = open("output/diagnostics.csv", "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "frame_label",
    "step", "sim_time_s",
    "punch_z_m", "punch_displacement_m",
    "n_particles",
    "max_strain", "mean_strain", "p99_strain",
    "J_min", "J_max", "J_mean", "J_std",
    "cfl_max",
    "pvel_max_ms", "pvel_mean_ms",
    "min_particle_mass_kg",
    "rotation_idx", "remesh_count",
    "warnings",
])

# export helper 
def export_frame(
    billet,
    scene,
    camera,
    cell_size:       float,
    step:            int,
    sim_t:           float,
    cur_punch_z:     float,
    frame_label:     str,
    rotation_idx:    int,
    remesh_count:    int,
    prev_max_strain: Optional[float],
    R_cumulative:    Optional[np.ndarray] = None,
) -> float:
    if R_cumulative is None:
        R_cumulative = np.eye(3, dtype=np.float64)
    """
    Capture particle state, compute diagnostics, write CSV row,
    write video frames, save PNG stills and NPY.

    Returns updated prev_max_strain value.
    """
    state = billet.get_state()
    pos   = get_pos(state)
    F     = get_F(state)
    vel   = get_vel(state)

    eps      = equiv_strain(F)
    J        = jacobians(F)
    max_eps  = float(eps.max())
    mean_eps = float(eps.mean())
    p99_eps  = float(np.percentile(eps, 99))
    J_min    = float(J.min())
    J_max    = float(J.max())
    J_mean   = float(J.mean())
    J_std    = float(J.std())

    cfl_max, spd = compute_cfl(vel, solver_cfg.dt, cell_size)
    vmax         = float(spd.max())
    vmean        = float(spd.mean())

    mpm_       = scene.sim.mpm_solver
    min_pm, _  = pmass(mpm_)

    p_disp = initial_punch_z - cur_punch_z

    flags = []
    if cfl_max >= CFL_CRITICAL:
        flags.append(f"CFL_CRITICAL({cfl_max:.3f})")
    elif cfl_max > CFL_WARN:
        flags.append(f"CFL_WARN({cfl_max:.3f})")
    if J_min <= 0.0:
        flags.append(f"J_INVERTED({J_min:.4f})")
    elif J_min < J_INVERT_WARN:
        flags.append(f"J_CRITICAL({J_min:.4f})")
    if J_max > J_EXPLODE_WARN:
        flags.append(f"J_EXPLODE({J_max:.3f})")
    if prev_max_strain is not None and prev_max_strain > 1e-8:
        r = max_eps / (prev_max_strain + 1e-12)
        if r > STRAIN_JUMP:
            flags.append(f"STRAIN_JUMP({r:.1f}x)")
    if max_eps > 1e-8:
        prev_max_strain = max_eps
    if vmax > PVEL_WARN:
        flags.append(f"PVEL_HIGH({vmax:.1f}m/s)")
    if not np.isnan(min_pm) and min_pm < PARTICLE_MASS_WARN:
        flags.append(f"PMASS_LOW({min_pm:.2e}kg)")

    flag_str = " | ".join(flags) if flags else "–"

    billet_height    = pos[:,2].max() - pos[:,2].min()
    height_reduction = cfg.billet_z - billet_height
    reduction_pct    = 100.0 * height_reduction / cfg.billet_z

    # print console
    print(
        f"[{frame_label}]  step={step:>7d}  t={sim_t:>8.5f}s  "
        f"pz={cur_punch_z:>9.5f}  maxε={max_eps:>9.5f}  "
        f"J∈[{J_min:>7.4f},{J_max:>7.4f}]  "
        f"CFL={cfl_max:>8.5f}  vmax={vmax:>7.3f}  "
        f"H={billet_height*1000:8.5f}mm  "
        f"ΔH={height_reduction*1000:8.5f}mm  "
        f"R={reduction_pct:5.1f}%  "
        f"{flag_str}"
    )

    # csv row
    csv_writer.writerow([
        frame_label,
        step, f"{sim_t:.7f}",
        f"{cur_punch_z:.6f}", f"{p_disp:.6f}",
        pos.shape[0],
        f"{max_eps:.6f}", f"{mean_eps:.6f}", f"{p99_eps:.6f}",
        f"{J_min:.6f}", f"{J_max:.6f}", f"{J_mean:.6f}", f"{J_std:.6f}",
        f"{cfl_max:.6f}",
        f"{vmax:.6f}", f"{vmean:.6f}",
        f"{min_pm:.4e}",
        rotation_idx, remesh_count,
        flag_str,
    ])
    csv_file.flush()

    # render and write videos and save npy

    np.save(f"output/npy/frame_{step:05d}.npy", pos)

    pbgr = render_particles(
        pos, eps, J, step, sim_t, cur_punch_z, flag_str, frame_label,
    )
    particle_video.write(pbgr)
    cv2.imwrite(f"output/particles/frame_{step:05d}.png", pbgr)

    img3d = render_solid_3d(
        pos, eps, J, step, sim_t, cur_punch_z, flag_str, frame_label,
        R_cumulative=R_cumulative,
    )
    imgxz = render_solid_xz(
        pos, eps, J, step, sim_t, cur_punch_z, flag_str, frame_label,
        R_cumulative=R_cumulative,
    )
    imgxy = render_solid_xy(
        pos, eps, J, step, sim_t, cur_punch_z, flag_str, frame_label,
        R_cumulative=R_cumulative,
    )

    solid_3d_video.write(img3d)
    solid_xz_video.write(imgxz)
    solid_xy_video.write(imgxy)
    cv2.imwrite(f"output/solid_3d/frame_{step:05d}.png", img3d)
    cv2.imwrite(f"output/solid_xz/frame_{step:05d}.png", imgxz)
    cv2.imwrite(f"output/solid_xy/frame_{step:05d}.png", imgxy)

    # Genesis camera frame
    try:
        img = camera.render(rgb=True)[0]
        if hasattr(img, "cpu"):
            img = img.cpu().numpy()
        img = np.asarray(img)
        while img.ndim > 3:
            img = img[0]
        if img.dtype != np.uint8:
            img = (np.clip(img, 0.0, 1.0) * 255).astype(np.uint8)
        if img.shape[1] != CAMERA_W or img.shape[0] != CAMERA_H:
            img = cv2.resize(
                img, (CAMERA_W, CAMERA_H), interpolation=cv2.INTER_AREA,
            )
        camera_video.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    except Exception as e:
        print(f"  [camera skip @ {step}: {e}]")

    return prev_max_strain


# INITIAL SCENE BUILD
################################################################
# build_scene() calls gs_init_fresh() internally, so the first
# call starts Genesis fresh (no prior destroy needed).

initial_billet_size   = np.array([cfg.billet_x, cfg.billet_y, cfg.billet_z])
initial_billet_center = np.array([0.0, 0.0, INITIAL_BILLET_CENTER_Z])

handles = build_scene(
    billet_center=initial_billet_center,
    billet_aabb_size=initial_billet_size,
    punch_z=INITIAL_PUNCH_Z,
)

scene    = handles["scene"]
billet   = handles["billet"]
punch    = handles["punch"]
table    = handles["table"]
camera   = handles["camera"]
cell_size = handles["cell_size"]

# Build Tool wrappers for the initial scene
punch_tool = Tool(punch, INITIAL_PUNCH_Z)
table_tool = Tool(table, table_z)

# Global remesh counter (total across all steps)
total_remesh_count = 0

# Debug geometry log 
print()
print("=" * 70)
print("  FORGING SIMULATION  –  HOT STEEL @ ~1150 °C  (v5)")
print("=" * 70)
print(f"  Billet:         {cfg.billet_x*1e3:.1f}×{cfg.billet_y*1e3:.1f}×{cfg.billet_z*1e3:.1f} mm")
print(f"  Cell size:      {cell_size*1e3:.2f} mm")
print(f"  Particle size:  {PARTICLE_SIZE*1e3:.3f} mm")
print(f"  Init gap:       {INIT_GAP*1e3:.3f} mm (BUG-3 fix, half a particle)")
print(f"  Tool morph:     gs.morphs.Mesh (OBJ, centred at origin)")
print(f"  Drive method:   set_velocity + set_position every step")
print(f"  Remesh mode:    geometry-only, no state injection ")
print()
print(f"  Anvil top Z:    {ANVIL_TOP_Z*1e3:.3f} mm")
print(f"  Billet bottom Z:{(INITIAL_BILLET_CENTER_Z - cfg.billet_z/2)*1e3:.3f} mm")
print(f"  Billet top Z:   {(INITIAL_BILLET_CENTER_Z + cfg.billet_z/2)*1e3:.3f} mm")
print(f"  Punch bottom Z: {(INITIAL_PUNCH_Z - cfg.punch_thickness/2)*1e3:.3f} mm")
print()
print(f"  Tool path steps: {len(TOOL_PATH)}")
for i, tp in enumerate(TOOL_PATH):
    if tp.kind == "punch":
        sc_ = tp.strike_cfg
        print(f"    [{i}] PUNCH  down={sc_.down_dist_m*1e3:.1f}mm  "
              f"up={sc_.up_dist_m*1e3:.1f}mm  "
              f"v={sc_.velocity_ms}m/s  "
              f"n_steps={sc_.n_steps}  "
              f"intra_remeshes={sc_.punch_remesh_times}")
    elif tp.kind == "rotate":
        print(f"    [{i}] ROTATE  {tp.angle_deg}° about {tp.axis}-axis  "
              f"pivot={'centroid' if tp.pivot_world is None else tp.pivot_world}")
print("=" * 70)
print()


###############################################################
# MAIN LOOP
###############################################################
# Iterates over TOOL_PATH steps.  For each punch step:
#   1. Mandatory pre-strike remesh (fresh geometry, clean state)
#   2. Run n_steps Genesis substeps, exporting at export_every
#      intervals.  Intra-strike remeshes are distributed
#      evenly across the n_steps window.
# For each rotate step:
#   1. Capture billet geometry (positions only) from Genesis
#   2. Apply rotation in Python (BilletState.apply_rotation)
#   3. Translate to correct set-down Z (with INIT_GAP, BUG-3)
#   4. Mandatory post-rotation remesh (new scene, new particles
#      seeded inside rotated AABB)
#   5. Export a single "jump frame" so the rotation is visible
###############################################################

# Running counters for frame label
rotation_idx   = 0    # number of rotations completed so far
strike_idx     = 0    # number of punch strikes completed so far
remesh_idx     = 0    # remesh within current strike (reset each strike)

# Global step counter (monotonically increasing across all strikes)
global_step       = 0
prev_max_strain   = None
current_R         = np.eye(3, dtype=np.float64)   # tracks cumulative rotation for export

# Current punch Z (maintained across remeshes so punch doesn't teleport)
cur_punch_z = INITIAL_PUNCH_Z

print(f"{'Frame Label':>30}  {'Step':>7}  {'time(s)':>8}  "
      f"{'punch_z':>9}  {'max_ε':>9}  {'J_min':>7}  {'J_max':>7}  "
      f"{'CFL_max':>8}  FLAGS")
print("-" * 100)


for path_step in TOOL_PATH:

    # Translate step
    if path_step.kind == "translate":

        print()
        print(f"  ── TRANSLATE STEP: "
              f"dx={path_step.translate_x*1e3:.2f}mm  "
              f"dy={path_step.translate_y*1e3:.2f}mm  "
              f"dz={path_step.translate_z*1e3:.2f}mm  "
              f"(world space) ──")

        # Capture current billet geometry from Genesis
        billet_state = capture_billet_geometry(billet)
        billet_state.rotation_idx = rotation_idx

        # Apply the world-space translation to particle positions
        billet_state.apply_world_translation(
            dx=path_step.translate_x,
            dy=path_step.translate_y,
            dz=path_step.translate_z,
        )

        # Re-seat the billet at the correct Z above the anvil (BUG-3 fix).
        # Only do this if the translation does not include a Z component —
        # if the user specified translate_z, honour it exactly and skip
        # the auto-setdown so they get the Z they asked for.
        if path_step.translate_z == 0.0:
            new_center_z = billet_state.compute_setdown_center_z(
                anvil_top_z=ANVIL_TOP_Z,
                particle_size=PARTICLE_SIZE,
                init_gap_frac=0.5,
            )
            billet_state.translate_to_z(new_center_z)

        print(f"     Billet centroid after translate: "
              f"X={billet_state.centroid()[0]*1e3:.2f}mm  "
              f"Y={billet_state.centroid()[1]*1e3:.2f}mm  "
              f"Z={billet_state.centroid()[2]*1e3:.2f}mm")

        # Remesh — Genesis seeds fresh particles at the new position
        lo_t, hi_t = billet_state.aabb()
        cur_punch_z = (
            hi_t[2]
            + cfg.punch_thickness / 2.0
            + 0.001   # 1 mm clearance
        )

        print(f"     Remeshing after translate "
              f"(rot{rotation_idx:02d}, mandatory)…")
        handles = remesh_scene(
            billet_state=billet_state,
            punch_z_world=cur_punch_z,
        )
        scene      = handles["scene"]
        billet     = handles["billet"]
        punch      = handles["punch"]
        table      = handles["table"]
        camera     = handles["camera"]
        cell_size  = handles["cell_size"]
        punch_tool = handles["punch_tool"]
        table_tool = handles["table_tool"]
        total_remesh_count += 1
        remesh_idx = 0

        # Export a jump frame to show the new position
        frame_label = f"rot{rotation_idx:02d}.translate"
        sim_t_jump  = global_step * solver_cfg.dt

        prev_max_strain = export_frame(
            billet=billet,
            scene=scene,
            camera=camera,
            cell_size=cell_size,
            step=global_step,
            sim_t=sim_t_jump,
            cur_punch_z=cur_punch_z,
            frame_label=frame_label,
            rotation_idx=rotation_idx,
            remesh_count=total_remesh_count,
            prev_max_strain=prev_max_strain,
            R_cumulative=current_R,
        )

        print(f"  ── TRANSLATE STEP COMPLETE ──")
        print()
        continue  # advance to next ToolPathStep
    
    # Rotation step
    if path_step.kind == "rotate":

        print()
        print(f"  ── ROTATE STEP: {path_step.angle_deg}° about "
              f"{path_step.axis}-axis ──")

        # Step 1: capture current billet geometry from Genesis
        billet_state = capture_billet_geometry(billet)
        billet_state.rotation_idx = rotation_idx

        # Step 2: convert particle cloud → surface mesh → rotate
        #         mesh → reseed particle positions from rotated mesh.
        #         R_cumulative is updated inside rotate_billet_via_mesh.
        pivot = (
            path_step.pivot_world
            if path_step.pivot_world is not None
            else billet_state.centroid()
        )
        billet_state = rotate_billet_via_mesh(
            billet_state=billet_state,
            angle_deg=path_step.angle_deg,
            axis=path_step.axis,
            pivot_world=pivot,
            voxel_res=32,
        )
        # rotate_billet_via_mesh increments rotation_idx inside
        # BilletState; sync the outer counter.
        rotation_idx = billet_state.rotation_idx

        # Step 3: translate to correct set-down Z above anvil (BUG-3 fix)
        new_center_z = billet_state.compute_setdown_center_z(
            anvil_top_z=ANVIL_TOP_Z,
            particle_size=PARTICLE_SIZE,
            init_gap_frac=0.5,
        )
        billet_state.translate_to_z(new_center_z)

        print(f"     New billet centroid Z after set-down: "
              f"{billet_state.centroid()[2]*1e3:.3f} mm")

        # Step 4: remesh — Genesis seeds fresh particles inside
        #         the rotated AABB.  Axis lines (R_cumulative)
        #         are now correctly rotated and will render
        #         with the new orientation.
        lo_r, hi_r = billet_state.aabb()
        cur_punch_z = (
            hi_r[2]
            + cfg.punch_thickness / 2.0
            + 0.001   # 1 mm clearance
        )

        print(f"     Remeshing after rotation "
              f"(rot{rotation_idx:02d}, mandatory)…")
        handles = remesh_scene(
            billet_state=billet_state,
            punch_z_world=cur_punch_z,
        )
        scene      = handles["scene"]
        billet     = handles["billet"]
        punch      = handles["punch"]
        table      = handles["table"]
        camera     = handles["camera"]
        cell_size  = handles["cell_size"]
        punch_tool = handles["punch_tool"]
        table_tool = handles["table_tool"]
        total_remesh_count += 1
        remesh_idx = 0   # reset for next strike context

        # Step 5: export a single "jump frame" so the rotation
        #         is visible in the video.  R_cumulative is now
        #         rotated so the axis lines appear in the new
        #         orientation immediately.
        frame_label = f"rot{rotation_idx:02d}.setdown"
        sim_t_jump  = global_step * solver_cfg.dt

        current_R = billet_state.R_cumulative.copy()
        prev_max_strain = export_frame(
            billet=billet,
            scene=scene,
            camera=camera,
            cell_size=cell_size,
            step=global_step,
            sim_t=sim_t_jump,
            cur_punch_z=cur_punch_z,
            frame_label=frame_label,
            rotation_idx=rotation_idx,
            remesh_count=total_remesh_count,
            prev_max_strain=prev_max_strain,
            R_cumulative=current_R,
        )

        print(f"  ── ROTATE STEP COMPLETE "
              f"(rotation_idx={rotation_idx}) ──")
        print()
        continue  # advance to next ToolPathStep


    # punch step 
    if path_step.kind != "punch":
        print(f"  [WARNING] Unknown ToolPathStep kind "
              f"'{path_step.kind}' — skipping.")
        continue

    sc_ = path_step.strike_cfg


    # Pre strike remesh:

    #    Destroys the current Genesis scene and starts a fresh
    #    simulation with the correct deformed billet geometry.
    #    Particle state (vel, F) is NOT transferred — the new
    #    scene begins at rest with identity deformation.
    #
    #    Has to be done before every strike 

    print()
    print(f"  ── PUNCH STEP {strike_idx+1}: "
          f"pre-strike remesh (mandatory) ──")

    billet_state = capture_billet_geometry(billet)
    billet_state.rotation_idx = rotation_idx

    # On the very first strike the particles are at the initial
    # Box positions with INIT_GAP already baked into
    # INITIAL_BILLET_CENTER_Z — do NOT recompute set-down Z
    # here, it would shift the billet.  Subsequent strikes
    # (after retraction) preserve whatever Z the billet is at.

    remesh_idx = 0   # reset remesh counter for the strike

    handles = remesh_scene(
        billet_state=billet_state,
        punch_z_world=cur_punch_z,
    )
    scene      = handles["scene"]
    billet     = handles["billet"]
    punch      = handles["punch"]
    table      = handles["table"]
    camera     = handles["camera"]
    cell_size  = handles["cell_size"]
    punch_tool = handles["punch_tool"]
    table_tool = handles["table_tool"]
    total_remesh_count += 1
    remesh_idx = 1   # count as remesh #1 of the strike

    print(f"     Pre-strike remesh complete "
          f"(total remeshes so far: {total_remesh_count})")

    # Schedule remeshing during strike
    # By evenly distributing punchRemeshTimes remeshes across n_steps for the strike
    
    n_steps         = sc_.n_steps
    intra_remeshes  = sc_.punch_remesh_times
    remesh_interval = n_steps // max(intra_remeshes + 1, 2)

    intra_remesh_steps = set(
        (i + 1) * remesh_interval
        for i in range(intra_remeshes)
        if (i + 1) * remesh_interval < n_steps
    )

    print(f"     Intra-strike remesh schedule "
          f"(local steps): {sorted(intra_remesh_steps)}")
    print()

    # stroke profile for this strike 
    profile = StrokeProfile(sc_)

    strike_local_time = 0.0

  
    # step loop for this strike
    for local_step in range(n_steps):

        frame_label = (
            f"rot{rotation_idx:02d}."
            f"strike{strike_idx+1:02d}."
            f"remesh{remesh_idx:02d}"
        )

        # Velocity from stroke profile at current local time
        vz = profile.velocity(strike_local_time)

        # Drive tools (CRITICAL: must happen BEFORE scene.step())
        punch_tool.update(velocity=vz, dt=solver_cfg.dt)
        table_tool.update(velocity=0.0, dt=solver_cfg.dt)

        cur_punch_z = punch_tool.z

        scene.step()

        global_step       += 1
        strike_local_time += solver_cfg.dt

        if local_step % solver_cfg.export_every == 0:
            prev_max_strain = export_frame(
                billet=billet,
                scene=scene,
                camera=camera,
                cell_size=cell_size,
                step=global_step,
                sim_t=global_step * solver_cfg.dt,
                cur_punch_z=cur_punch_z,
                frame_label=frame_label,
                rotation_idx=rotation_idx,
                remesh_count=total_remesh_count,
                prev_max_strain=prev_max_strain,
                R_cumulative=current_R,
            )

        # remesh during strike
        if local_step in intra_remesh_steps:

            remesh_idx += 1
            frame_label = (
                f"rot{rotation_idx:02d}."
                f"strike{strike_idx+1:02d}."
                f"remesh{remesh_idx:02d}"
            )

            print(f"  ── Intra-strike remesh: {frame_label} "
                  f"(local_step={local_step}) ──")

            # Capture geometry (positions) from current scene.
            # No particle state velocity transfer, new scene starts fresh.
            billet_state = capture_billet_geometry(
                billet_entity=billet,
                existing_state=BilletState(
                    pos=np.zeros((1, 3)),   # placeholder(overwritten)
                    R_cumulative=np.eye(3),
                    rotation_idx=rotation_idx,
                ),
            )

            handles = remesh_scene(
                billet_state=billet_state,
                punch_z_world=cur_punch_z,
            )
            scene      = handles["scene"]
            billet     = handles["billet"]
            punch      = handles["punch"]
            table      = handles["table"]
            camera     = handles["camera"]
            cell_size  = handles["cell_size"]
            punch_tool = handles["punch_tool"]
            table_tool = handles["table_tool"]
            total_remesh_count += 1

            # Position punch after rebuild.
            # Velocity is set to zero will be overwritten by punch_tool.update() 
            # in the next loop iteration.
            punch_tool.reset_z(cur_punch_z)
            table_tool.reset_z(table_z)

            print(f"     Intra-strike remesh complete "
                  f"(total remeshes: {total_remesh_count})")

    strike_idx += 1
    print()
    print(f"  ── PUNCH STRIKE {strike_idx} COMPLETE ──")
    print()


# export video 
camera_video.release()
particle_video.release()
solid_3d_video.release()
solid_xz_video.release()
solid_xy_video.release()
csv_file.close()

print()
print("=" * 70)
print("DONE")
print(f"  Total Genesis steps executed : {global_step}")
print(f"  Total remeshes performed     : {total_remesh_count}")
print(f"  Total rotations performed    : {rotation_idx}")
print()
print("  output/forge_camera.mp4       – Genesis camera animation")
print("  output/forge_particles.mp4    – strain / Jacobian scatter")
print("  output/forge_solid_3d.mp4     – 3D solid body deformation")
print("  output/forge_solid_xz.mp4     – XZ side profile")
print("  output/forge_solid_xy.mp4     – XY top view")
print("  output/diagnostics.csv        – per-step metrics")
print("  output/npy/                   – particle position arrays")
print("  output/particles/             – particle PNG stills")
print("  output/solid_3d/              – 3D solid PNG stills")
print("  output/solid_xz/              – XZ solid PNG stills")
print("  output/solid_xy/              – XY solid PNG stills")
print("=" * 70)