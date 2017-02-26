model;

var x1;
var x2;

minimize function:
	100 * (x2 - x1^2)^2 + (1 - x1)^2;
	
subject to dontChangeSolutionInequalityConstraint:
	x1 <= 2;