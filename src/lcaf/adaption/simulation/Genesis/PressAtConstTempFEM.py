import genesis as gs
import numpy as np

#genesis cannot do plastic deformation forging simulation with FEM since it was designed for robotics 
gs.init()

INCH = 0.0254

BILLET_X = 1 * INCH
BILLET_Y = 1 * INCH
BILLET_Z = 3 * INCH

PUNCH_TRAVEL = 1 * INCH
N_STEPS = 2000

scene = gs.Scene(

    sim_options=gs.options.SimOptions(
        dt=5e-4,
        substeps=10,
    ),

    fem_options=gs.options.FEMOptions(
        use_implicit_solver=False,
    ),
)

# --------------------------------------------------
# Billet
# --------------------------------------------------

billet = scene.add_entity(

    morph=gs.morphs.Box(
        size=(BILLET_X, BILLET_Y, BILLET_Z),
        pos=(0, 0, 0),
    ),

    material=gs.materials.FEM.Elastic(
        E=150e9,
        nu=0.30,
        rho=7800,
        model="linear",
    ),
)

# --------------------------------------------------
# Bottom die
# --------------------------------------------------

bottom_die = scene.add_entity(

    morph=gs.morphs.Box(
        size=(4*INCH, 4*INCH, 0.5*INCH),
        pos=(0, 0, -BILLET_Z/2 - 0.25*INCH),
    ),

    material=gs.materials.Rigid(),
)

# --------------------------------------------------
# Top die
# --------------------------------------------------

top_die = scene.add_entity(

    morph=gs.morphs.Box(
        size=(4*INCH, 4*INCH, 0.5*INCH),

        pos=(
            0,
            0,
            BILLET_Z/2 + 0.25*INCH,
        ),
    ),

    material=gs.materials.Rigid(),
)

scene.build()

# --------------------------------------------------
# Compression
# --------------------------------------------------

start_z = BILLET_Z/2 + 0.25*INCH

for step in range(N_STEPS):

    frac = step / N_STEPS

    die_z = start_z - frac * PUNCH_TRAVEL

    top_die.set_pos(
        np.array([0.0, 0.0, die_z])
    )

    scene.step()

print("Finished")