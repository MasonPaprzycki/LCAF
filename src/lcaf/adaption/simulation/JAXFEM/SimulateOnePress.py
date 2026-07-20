import os
import numpy as onp
import jax.numpy as np
import meshio

from jax_fem.problem import Problem
from jax_fem.solver import solver
from jax_fem.generate_mesh import (
    box_mesh_gmsh,
    get_meshio_cell_type,
    Mesh,
)

# ============================================================
# GEOMETRY
# ============================================================

LENGTH = 12.0
WIDTH = 4.0
HEIGHT = 4.0

TARGET_DISPLACEMENT = -1.0

NX = 1
NY = 1
NZ = 36

ele_type = "HEX8"
cell_type = get_meshio_cell_type(ele_type)

# ============================================================
# MATERIAL
# ============================================================

E = 30e6
NU = 0.29

mu = E / (2.0 * (1.0 + NU))
lam = E * NU / ((1.0 + NU) * (1.0 - 2.0 * NU))

# ============================================================
# MESH
# ============================================================

meshio_mesh = box_mesh_gmsh(
    Nx=NX,
    Ny=NY,
    Nz=NZ,
    domain_x=LENGTH,
    domain_y=WIDTH,
    domain_z=HEIGHT,
    data_dir="mesh",
    ele_type=ele_type,
)

mesh = Mesh(
    meshio_mesh.points,
    meshio_mesh.cells_dict[cell_type],
)

# ============================================================
# BOUNDARY CONDITIONS
# ============================================================

def bottom_face(x):
    return np.isclose(x[2], 0.0, atol=1e-6)

def top_face(x):
    return np.isclose(x[2], HEIGHT, atol=1e-6)

def zero(x):
    return 0.0

# ============================================================
# PROBLEM
# ============================================================

class LinearElasticProblem(Problem):

    def get_tensor_map(self):

        def stress(u_grad):

            eps = 0.5 * (u_grad + u_grad.T)

            sigma = (
                lam * np.trace(eps) * np.eye(3)
                + 2.0 * mu * eps
            )

            return sigma

        return stress


# ============================================================
# OUTPUT DIRECTORY
# ============================================================

output_dir = "results"
os.makedirs(output_dir, exist_ok=True)

# ============================================================
# LOAD STEPS
# ============================================================

n_steps = 21

points = onp.asarray(mesh.points)
cells = onp.asarray(mesh.cells)

pvd_entries = []

for step in range(n_steps):

    load_factor = step / (n_steps - 1)
    current_disp = load_factor * TARGET_DISPLACEMENT

    def top_disp(x, value=current_disp):
        return value

    problem = LinearElasticProblem(
        mesh=mesh,
        vec=3,
        dim=3,
        ele_type=ele_type,
        dirichlet_bc_info=[
            [bottom_face, bottom_face, bottom_face, top_face],
            [0, 1, 2, 2],
            [zero, zero, zero, top_disp],
        ],
    )

    sol = solver(problem)
    u = sol[0]

    u_np = onp.asarray(u)

    disp_mag = onp.linalg.norm(u_np, axis=1)

    filename = f"compression_{step:03d}.vtu"
    filepath = os.path.join(output_dir, filename)

    meshio.Mesh(
        points=points,
        cells=[
            ("hexahedron", cells)
        ],
        point_data={
            "displacement": u_np,
            "displacement_magnitude": disp_mag,
            "deformed_coordinates": points + u_np,
        },
    ).write(filepath)

    pvd_entries.append(
        f'    <DataSet timestep="{load_factor}" '
        f'part="0" file="{filename}"/>'
    )

    print(
        f"Step {step:03d}/{n_steps-1:03d} "
        f"disp={current_disp:.6f}"
    )

# ============================================================
# WRITE PVD COLLECTION
# ============================================================

pvd_file = os.path.join(output_dir, "compression.pvd")

with open(pvd_file, "w") as f:
    f.write('<?xml version="1.0"?>\n')
    f.write(
        '<VTKFile type="Collection" '
        'version="0.1" byte_order="LittleEndian">\n'
    )
    f.write("  <Collection>\n")

    for entry in pvd_entries:
        f.write(entry + "\n")

    f.write("  </Collection>\n")
    f.write("</VTKFile>\n")

print(f"\nExported {n_steps} VTU files")
print(f"PVD collection: {pvd_file}")