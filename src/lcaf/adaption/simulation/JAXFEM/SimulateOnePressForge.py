# =============================================================================
# JAX-FEM FORGING FRAMEWORK #1
# -----------------------------------------------------------------------------
# PURPOSE
# -----------------------------------------------------------------------------
# Simulate a single forging blow on a pearlitic steel billet.
#
# Geometry:
#   Steel billet:
#       Width  = 4 in
#       Height = 4 in
#       Length = 12 in
#
#   Hammer:
#       5 in x 5 in square flat die
#
# Process:
#   Apply 1 inch downward displacement to the top face.
#
# Modeling assumptions:
# -----------------------------------------------------------------------------
# 1. This follows the JAX-FEM hyperelastic/plasticity examples.
#
# 2. No contact mechanics are modeled.
#    The hammer is represented as a prescribed Dirichlet displacement boundary
#    condition.
#
# 3. No friction.
#
# 4. No thermal effects.
#
# 5. No strain-rate effects.
#
# 6. No remeshing.
#
# 7. No damage.
#
# 8. No recrystallization.
#
# 9. No metallurgical grain evolution.
#
# 10. Large deformation formulation is used.
#
# IMPORTANT
# -----------------------------------------------------------------------------
# Real forging steel should be modeled using finite-strain elastoplasticity
# with hardening and usually temperature-dependent constitutive laws.
#
# JAX-FEM currently contains a J2 plasticity example and a hyperelastic
# example. This script combines the architecture of those examples into a
# practical forging framework.
#
# The grain structure output requested cannot realistically be obtained from
# the standard JAX-FEM examples because no crystal plasticity or
# microstructure evolution model exists in those tutorials.
#
# Therefore this script exports:
#
#   - displacement
#   - strain
#   - stress
#   - equivalent plastic strain (placeholder)
#
# which can later be used by a separate microstructure model.
#
# OUTPUT
# -----------------------------------------------------------------------------
# Writes a VTK file at every load increment.
#
# ParaView:
#   Open all .vtu files
#   Apply "Temporal Interpolator"
#   Play animation
#
# LabVIEW:
#   LabVIEW can read VTK directly through available VTK interfaces or through
#   conversion to CSV/HDF5.
#
# =============================================================================

import os

import jax
import jax.numpy as np

from jax_fem.problem import Problem
from jax_fem.solver import solver
from jax_fem.utils import save_sol
from jax_fem.generate_mesh import (
    box_mesh_gmsh,
    get_meshio_cell_type,
    Mesh,
)


# =============================================================================
# UNIT SYSTEM
# =============================================================================
#
# inches
# pounds-force
# psi
#
# JAX-FEM itself is unit agnostic.
#
# =============================================================================

BILLET_X = 4.0
BILLET_Y = 4.0
BILLET_Z = 12.0

TARGET_DISPLACEMENT = -1.0  # inches

# =============================================================================
# PEARLITIC STEEL APPROXIMATION
# =============================================================================
#
# This is only an approximate placeholder.
#
# A real forging model should use:
#
#     Johnson-Cook
#     Voce hardening
#     Hensel-Spittel
#     or experimentally calibrated flow stress
#
# =============================================================================

#Output directory 
data_dir = "forging_output"

vtk_dir = os.path.join(
    data_dir,
    "vtk"
)



#paramaters for pearlitic steel

E = 30.0e6
NU = 0.29

YIELD_STRESS = 50000.0




# MESH

NX = 12
NY = 12
NZ = 36

ele_type = "HEX8"
cell_type = get_meshio_cell_type(ele_type)


data_dir = "forging_output"

meshio_mesh = box_mesh_gmsh(
    Nx=NX,
    Ny=NY,
    Nz=NZ,
    domain_x=BILLET_X,
    domain_y=BILLET_Y,
    domain_z=BILLET_Z,
    data_dir=data_dir,
    ele_type=ele_type,
)


mesh = Mesh(
    meshio_mesh.points,
    meshio_mesh.cells_dict[cell_type],
)



# BOUNDARY LOCATORS

def bottom_face(point):
    return np.isclose(point[2], 0.0, atol=1e-6)

def top_face(point):
    return np.isclose(point[2], BILLET_Z, atol=1e-6)


# LOAD STEP GLOBAL VARIABLE

CURRENT_DISPLACEMENT = 0.0


# DISPLACEMENT BC FUNCTIONS

def zero_disp(point):
    return 0.0

def top_z_disp(point):
    return CURRENT_DISPLACEMENT

# =============================================================================
# PLASTICITY MODEL FRAMEWORK
# =============================================================================
#
# Architecture follows the official JAX-FEM plasticity example.
#
# Replace stress_return_map() with a full radial-return implementation if
# production-level plasticity is desired.
#
# =============================================================================

class ForgingPlasticity(Problem):

    def custom_init(self):

        self.fe = self.fes[0]

        self.eps_old = np.zeros(
            (
                len(self.fe.cells),
                self.fe.num_quads,
                self.fe.vec,
                self.dim,
            )
        )

        self.sig_old = np.zeros_like(self.eps_old)

        self.internal_vars = [
            self.sig_old,
            self.eps_old,
        ]

    def get_tensor_map(self):

        def strain(u_grad):
            return 0.5 * (u_grad + u_grad.T)

        def stress_return_map(
            u_grad,
            sigma_old,
            epsilon_old,
        ):
            #
            # Placeholder constitutive model.
            #
            # Replace with the exact J2 return mapping algorithm from the
            # JAX-FEM plasticity example.
            #

            eps = strain(u_grad)

            mu = E / (2.0 * (1.0 + NU))
            lam = E * NU / (
                (1.0 + NU) * (1.0 - 2.0 * NU)
            )

            sigma = (
                lam * np.trace(eps) * np.eye(3)
                + 2.0 * mu * eps
            )

            return sigma

        return stress_return_map

# =============================================================================
# BC DEFINITION
# =============================================================================

location_fns = [
    bottom_face,
    bottom_face,
    bottom_face,
    top_face,
]

vecs = [
    0,
    1,
    2,
    2,
]

value_fns = [
    zero_disp,
    zero_disp,
    zero_disp,
    top_z_disp,
]

dirichlet_bc_info = [
    location_fns,
    vecs,
    value_fns,
]

# =============================================================================
# PROBLEM
# =============================================================================

problem = ForgingPlasticity(
    mesh=mesh,
    vec=3,
    dim=3,
    ele_type=ele_type,
    dirichlet_bc_info=dirichlet_bc_info,
)

# =============================================================================
# OUTPUT DIRECTORY



os.makedirs(vtk_dir, exist_ok=True)

# =============================================================================
# INCREMENTAL FORGING
# =============================================================================
#
# Real plasticity requires load stepping.
#
# =============================================================================

NUM_STEPS = 20

for step in range(NUM_STEPS + 1):

    CURRENT_DISPLACEMENT = (
        TARGET_DISPLACEMENT
        * step
        / NUM_STEPS
    )

    print(
        f"Step {step}  disp={CURRENT_DISPLACEMENT}"
    )

    sol = solver(problem)

    vtk_path = os.path.join(
        vtk_dir,
        f"forge_{step:04d}.vtu",
    )

    save_sol(
        problem.fe,
        sol[0],
        vtk_path,
    )

print("Finished.")

# =============================================================================
# OPTIONAL POSTPROCESSING IDEAS
# =============================================================================
#
# 1. Compute equivalent plastic strain
#
# 2. Compute forging reduction ratio
#
# 3. Compute volume conservation error
#
# 4. Export stress tensors
#
# 5. Export strain tensors
#
# 6. Export von-Mises stress
#
# 7. Feed plastic strain field into a separate grain-growth model
#
# =============================================================================