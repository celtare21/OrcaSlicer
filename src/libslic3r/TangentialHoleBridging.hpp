#ifndef slic3r_TangentialHoleBridging_hpp_
#define slic3r_TangentialHoleBridging_hpp_

namespace Slic3r {

class PrintObject;
class LayerRegion;

class TangentialHoleBridging {
public:
    // ORCA: Main entry point to apply the tangential sacrificial bridging
    // Generates crossed struts for counterbore holes.
    static void apply(PrintObject* print_object);
    
    // ORCA: Generate tangential crossed struts for holes in bridge layers
    // to support the perimeter of the hole from falling down.
    static void apply_to_bridges(LayerRegion* region);
};


} // namespace Slic3r

#endif // slic3r_TangentialHoleBridging_hpp_
