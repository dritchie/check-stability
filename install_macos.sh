# install octomap (dependency for FCL)
brew install octomap

# install libccd (dependency for FCL)
brew install libccd

# install FCL
brew install fcl

# install FCL python bindings
CFLAGS="-stdlib=libc++" pip install python-fcl

# install trimesh
pip install trimesh

# install bullet
pip install pybullet

# install pandas
pip install pandas
