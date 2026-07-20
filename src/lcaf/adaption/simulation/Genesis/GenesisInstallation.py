import genesis as gs

#--------------------------------------------------------------------
#To install genesis, run the following commands in your terminal:
#--------------------------------------------------------------------

# 1. Install PyTorch with the appropriate command for your platform (see table below).

#---------------------------------------------------------------------------------------
#platform     ###      command
#---------------------------------------------------------------------------------------
#NVIDIA GPU (CUDA)	pip install torch --index-url https://download.pytorch.org/whl/cu121
#AMD GPU (ROCm)	pip install torch --index-url https://download.pytorch.org/whl/rocm6.0
#Apple Silicon	pip install torch (MPS backend included)
#CPU Only	pip install torch --index-url https://download.pytorch.org/whl/cpu
#---------------------------------------------------------------------------------------

#2. Install the genesis package using pip: 

#---------------------------------------------------------------------------------------
#pip install genesis-world
#---------------------------------------------------------------------------------------
 
#3. YOU ARE NOW DONE


# After installation, you can run the following code to verify that Genesis is working correctly:


# Initialize Genesis with automatic backend selection
gs.init()
 
print(f"Backend: {gs.backend}")
print(f"Device: {gs.device}")
print(f"Precision: {gs.np_float}")
print(f"Zero-copy enabled: {gs.use_zerocopy}")
 
# Create a simple scene object and build it
scene = gs.Scene()
scene.build()