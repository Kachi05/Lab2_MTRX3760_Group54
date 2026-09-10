//-----------------------------------------------------------------------------
// area_node.cpp
//
// Tests the polygon plugins using Pluginlib.
// A Triangle, Square and House are loaded and initialised with a side length
// of 10.0 before their areas are printed to the terminal.
//
// MTRX3760 Lab 2 - A4
//-----------------------------------------------------------------------------

//---Includes------------------------------------------------------------------

#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>


//---Main----------------------------------------------------------------------

// Loads each polygon plugin and tests its area calculation.
int main( int argc, char ** argv )
{
    (void) argc;
    (void) argv;


    // Create a loader for classes derived from RegularPolygon.
    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
        "polygon_base",
        "polygon_base::RegularPolygon" );


    try
    {
        //---Triangle---

        std::shared_ptr<polygon_base::RegularPolygon> triangle =
            poly_loader.createSharedInstance(
                "awesome_triangle" );

        triangle->initialize( 10.0 );


        //---Square---

        std::shared_ptr<polygon_base::RegularPolygon> square =
            poly_loader.createSharedInstance(
                "polygon_plugins::Square" );

        square->initialize( 10.0 );


        //---House---

        std::shared_ptr<polygon_base::RegularPolygon> house =
            poly_loader.createSharedInstance(
                "house" );

        house->initialize( 10.0 );


        //---Results---

        printf(
            "Triangle area: %.2f\n",
            triangle->area() );

        printf(
            "Square area: %.2f\n",
            square->area() );

        printf(
            "House area: %.2f\n",
            house->area() );
    }

    catch( pluginlib::PluginlibException & ex )
    {
        printf(
            "The plugin failed to load for some reason. Error: %s\n",
            ex.what() );
    }


    return 0;
}
