# A Process Planner for 3-Axis CNC Mills

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