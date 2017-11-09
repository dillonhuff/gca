# A Process Planner for 3-Axis CNC Mills

GCA is a basic process planner that automatically generates machining plans for 3-axis
mills. Below is an example part that was manufactured using a process plan produced
by GCA.

CAD Model                  |  Final Part
:-------------------------:|:-------------------------:
![Screenshot](/images/CircleWithFilletSide.jpg)  |  ![Screenshot](/images/Half_sphere_teaser_part.jpg)

### Inputs
1. An STL file containing the part to be manufactured
2. A list of the available cutting tools for the mill. GCA supports flat nosed and ball nosed tools
3. The dimensions of the stock
4. The dimensions of the vice that will be used, and the heights of any available parallel plats

### Process Planning Pipeline
1. Stock Alignment
2. Feature Recognition
3. Setup Selection
4. Toolpath Generation

### Output
GCA produces a list of setups that when carried out on a 3-axis mill will manufacture
the given part from the given stock. Each setup is complete with a description
of how to place the part in the vice, which parallel plates to use (if any),
ready to run G-code programs for each toolpath, and instructions on which tool to
use in each path.

To see some examples of how to use GCAs C++ API see the tests in [https://github.com/dillonhuff/gca/blob/master/test/mesh_to_gcode_tests.cpp](https://github.com/dillonhuff/gca/blob/master/test/mesh_to_gcode_tests.cpp)

## Testing

GCA has been used to make several real parts out of 6061 aluminum on an Emco F1
3-axis mill. It should work well on prismatic parts, and parts that are mostly
prismatic with isolated freeform surfaces

## Limitations

GCA supports only flat nosed and ball nosed end mills. Special tools like chamfers
and corner-rounding end-mills are not supported.

GCA only supports cuboid stocks.

The only fixture type that is supported is vices.
