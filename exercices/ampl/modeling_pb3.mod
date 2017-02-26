model;

var x1;
var x2;

minimize function:
	(x1 - 2)^2 + (x2 -1)^2;
	
subject to zeroConstraint:
	x1^2 - x2 <= 0;
	
subject to twoConstraint:
	x1 + x2 <= 2;