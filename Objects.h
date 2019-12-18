#pragma once
class SquareObject
{
private:
	double x_cood;
	double y_cood;
	double length;

public:
	SquareObject(double len, double xCood, double yCood)
	{
		x_cood = xCood;
		y_cood = yCood;
		length = len;
	}

	double getX_cood()
	{
		return x_cood;
	}

	double getY_cood()
	{
		return y_cood;
	}

	double getLength()
	{
		return length;
	}
};