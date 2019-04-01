## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD  
  
from distutils.core import setup  
from catkin_pkg.python_setup import generate_distutils_setup  
  
# fetch values from package.xml  
setup_args = generate_distutils_setup(  
    packages=['ros_robodk_post_processors', 'ros_robodk_post_processors.robodk_post_processors'],
    package_dir={'': 'src'},  
)  
  
setup(**setup_args) 
