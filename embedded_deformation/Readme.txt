Data generation

Coordinate system: XYZ in the original model. Center of mass is the orginal point.

Input: Model ; Center of mass ; Volumn of original model; Spatial direction relationship between observation point and model; Obseration step;

Iterate: Rotate the model by "Center of mass" and X axis by 2pi/n degree.
	 Select 6 points( Min & Max point by cutting model with XY, YZ, XZ planes(within a selected region)); Randomly move Min XY plane point.
	 Deform the rotated model by these 6 points.

Output: Rotated and deformed model and corresponding depth.