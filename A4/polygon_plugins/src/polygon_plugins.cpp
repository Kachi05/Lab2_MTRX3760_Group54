//-----------------------------------------------------------------------------
// polygon_plugins.cpp
//
// Implements the polygon plugins used by the Pluginlib tutorial.
// The original Square and Triangle plugins are extended with a House plugin
// containing both a square and a triangle.
//
// MTRX3760 Lab 2 - A4
//-----------------------------------------------------------------------------

//---Includes------------------------------------------------------------------

#include <polygon_base/regular_polygon.hpp>

#include <cmath>


//---Polygon Plugins-----------------------------------------------------------

namespace polygon_plugins
{

//-----------------------------------------------------------------------------
// Square represents a regular square with a specified side length.
//-----------------------------------------------------------------------------
class Square : public polygon_base::RegularPolygon
{
    public:

        // Sets the side length of the square.
        void initialize( double side_length ) override
        {
            side_length_ = side_length;
        }


        // Returns the area of the square.
        double area() override
        {
            return side_length_ * side_length_;
        }


    protected:

        double side_length_;
};


//-----------------------------------------------------------------------------
// Triangle represents an equilateral triangle with a specified side length.
//-----------------------------------------------------------------------------
class Triangle : public polygon_base::RegularPolygon
{
    public:

        // Sets the side length of the triangle.
        void initialize( double side_length ) override
        {
            side_length_ = side_length;
        }


        // Returns the area of the triangle.
        double area() override
        {
            return 0.5 * side_length_ * getHeight();
        }


        // Calculates the height of the equilateral triangle.
        double getHeight()
        {
            return std::sqrt(
                ( side_length_ * side_length_ )
                - ( ( side_length_ / 2.0 )
                * ( side_length_ / 2.0 ) ) );
        }


    protected:

        double side_length_;
};


//-----------------------------------------------------------------------------
// House represents a shape containing a square base and triangular roof.
// Both parts are initialised using the same side length.
//-----------------------------------------------------------------------------
class House : public polygon_base::RegularPolygon
{
    public:

        // Initialises both parts of the house using the same side length.
        void initialize( double side_length ) override
        {
            square_.initialize( side_length );
            triangle_.initialize( side_length );
        }


        // Returns the combined area of the square and triangle.
        double area() override
        {
            return square_.area()
                + triangle_.area();
        }


    private:

        //---House components---

        Square square_;
        Triangle triangle_;
};

} // namespace polygon_plugins


//---Plugin Exports------------------------------------------------------------

#include <pluginlib/class_list_macros.hpp>

// Register each polygon class so it can be loaded through Pluginlib.
PLUGINLIB_EXPORT_CLASS(
    polygon_plugins::Square,
    polygon_base::RegularPolygon )

PLUGINLIB_EXPORT_CLASS(
    polygon_plugins::Triangle,
    polygon_base::RegularPolygon )

PLUGINLIB_EXPORT_CLASS(
    polygon_plugins::House,
    polygon_base::RegularPolygon )
