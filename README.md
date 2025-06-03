# ros2_template_gen

Template code to generate a ros2 python package

This generates a param YAML, launch file with this YAML and an executable. Once run the params are loaded. Currently only 1 set of launch, params, exe are generated.

### Steps
- Clone the repo into the ros2 `<workspace>/src` folder
- `cd` into the repo folder
- Run `configure.py`
- Provide the following
  - Package name
  - Base python file (This is the python file that contains your main executable). **This is NOT the name of your node**.
  - Executable name (Name of the node executable which is usually defined in the `setup.py`)
  - Node name (Useful for params)
 

Once the creation is done simply use `colcon build --packages-selct <package>`. You will receive warnings that this "template package" does not follow naming conventions.
Launch your code using `ros2 launch <package> start.launch.py`
