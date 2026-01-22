#include "TangentialHoleBridging.hpp"
#include "Print.hpp"
#include "PrintConfig.hpp"
#include "Layer.hpp"
#include "ClipperUtils.hpp"
#include "Geometry.hpp"

namespace Slic3r {

void TangentialHoleBridging::apply(PrintObject* print_object)
{
    // Check if any region has the feature enabled
    bool enabled = false;
    for (size_t i = 0; i < print_object->num_printing_regions(); ++i) {
        if (print_object->printing_region(i).config().counterbore_hole_bridging == chbTangential) {
            enabled = true;
            break;
        }
    }
    if (!enabled) return;

    const std::vector<Layer*>& m_layers = print_object->layers();

    // Iterate layers (bottom to top) to find counterbore shelf overhangs.
    // Standard cross-hatch sacrificial bridge uses 2 layers (N-1, N-2).
    for (size_t i = 2; i < m_layers.size(); ++i) {
        Layer* layer_shelf = m_layers[i];
        Layer* layer_below = m_layers[i - 1];

        // Find Overhangs: Material in Shelf that is NOT in Below.
        ExPolygons overhangs = diff_ex(layer_shelf->lslices, layer_below->lslices);
        if (overhangs.empty()) continue;

        for (const ExPolygon& ov : overhangs) {
            // Calculate Anchor Zone (Shelf Contour + 3mm)
            // This ensures bridges follow the curve and anchor into the wall.
            double anchor_dist = scale_(3.0);
            Polygons shelf_contour;
            shelf_contour.push_back(ov.contour);
            // Offset contour outwards to define the valid area for bridging
            ExPolygons anchor_zone = offset_ex(shelf_contour, anchor_dist);
            
            // Iterate over all holes in the overhang.
            for (const Polygon& hole : ov.holes) {
                BoundingBox hole_bbox = hole.bounding_box();
                Point center = hole_bbox.center();
                coord_t rx = hole_bbox.size().x() / 2;
                coord_t ry = hole_bbox.size().y() / 2;

                // Ignore tiny holes (artefacts or too small for bridging logic)
                if (rx < scale_(1.0) || ry < scale_(1.0)) continue;

                // Dynamic overlap to ensure large holes are bridged correctly (15% of radius or min 0.5mm)
                double overlap = std::max(scale_(0.5), (double)rx * 0.15);
                
                // Helper to create a rectangle
                auto make_rect = [](coord_t x_min, coord_t y_min, coord_t x_max, coord_t y_max) {
                    Polygon p;
                    p.points.resize(4);
                    p.points[0] = Point(x_min, y_min);
                    p.points[1] = Point(x_max, y_min);
                    p.points[2] = Point(x_max, y_max);
                    p.points[3] = Point(x_min, y_max);
                    return p;
                };

                coord_t c_x = center.x();
                coord_t c_y = center.y();
                coord_t ovl = static_cast<coord_t>(overlap);
                coord_t huge_len = scale_(1000.0); // Effectively infinite length for clipping

                // 1. Vertical Bars (Left / Right) -> Layer N-1
                // Position: Tangent to Inner Hole X
                ExPolygons bridges_vertical;
                {
                    // Left Bar
                    // Extend from Tangent (Inner) to Infinite (Outer)
                    bridges_vertical.push_back(ExPolygon(make_rect(
                        c_x - rx - huge_len, c_y - huge_len,
                        c_x - rx + ovl, c_y + huge_len
                    )));
                    // Right Bar
                    bridges_vertical.push_back(ExPolygon(make_rect(
                        c_x + rx - ovl, c_y - huge_len,
                        c_x + rx + huge_len, c_y + huge_len
                    )));
                }

                // 2. Horizontal Bars (Top / Bottom) -> Layer N-2
                // Position: Tangent to Inner Hole Y
                ExPolygons bridges_horizontal;
                {
                    // Bottom Bar
                    bridges_horizontal.push_back(ExPolygon(make_rect(
                        c_x - huge_len, c_y - ry - huge_len,
                        c_x + huge_len, c_y - ry + ovl
                    )));
                    // Top Bar
                    bridges_horizontal.push_back(ExPolygon(make_rect(
                        c_x - huge_len, c_y + ry - ovl,
                        c_x + huge_len, c_y + ry + huge_len
                    )));
                }

                // Clip bars to the Anchor Zone (Shelf + 3mm)
                // This trims the "infinite" bars to the exact shape of the counterbore wall + anchor depth.
                bridges_vertical = intersection_ex(bridges_vertical, anchor_zone);
                bridges_horizontal = intersection_ex(bridges_horizontal, anchor_zone);
                
                auto integrate_polys = [&](Layer* layer, const ExPolygons& polys_to_add) {
                    if (layer->region_count() == 0) return;
                    
                    // 1. Clip to Layer Footprint (Part Outline)
                    // Prevents bridges from sticking out of the component if the wall is thin.
                    Polygons layer_outlines;
                    for (const ExPolygon& exp : layer->lslices) {
                         layer_outlines.push_back(exp.contour);
                    }
                    ExPolygons footprint = union_ex(layer_outlines);
                    ExPolygons clipped_polys = intersection_ex(polys_to_add, footprint);
                    
                    if (clipped_polys.empty()) return;

                    // Strategy: Negative Mask.
                    // The bridges are allowed everywhere EXCEPT in "Neighbor Holes".
                    
                    Polygons neighbor_holes;
                    for (const ExPolygon& exp : layer->lslices) {
                        for (const Polygon& h : exp.holes) {
                            // Check if this hole is the "Current Hole" we are bridging.
                            // We use the center point to check.
                            if (!h.contains(center)) {
                                neighbor_holes.push_back(h);
                            }
                        }
                    }
                    
                    ExPolygons forbidden_area;
                    if (!neighbor_holes.empty()) {
                        forbidden_area = union_ex(neighbor_holes);
                    }
                    
                    // Clip: Bridge MINUS Neighbors
                    ExPolygons valid_polys;
                    if (forbidden_area.empty()) {
                        valid_polys = clipped_polys;
                    } else {
                        valid_polys = diff_ex(clipped_polys, forbidden_area);
                    }
                    
                    if (valid_polys.empty()) return;

                    // Merge into Region 0
                    LayerRegion* region = layer->get_region(0);
                    ExPolygons existing_polys = to_expolygons(region->slices.surfaces);
                    ExPolygons merged_polys = union_ex(existing_polys, valid_polys);
                    
                    region->slices.surfaces.clear();
                    region->slices.append(merged_polys, stInternal);
                    
                    // Update global lslices
                    layer->lslices = union_ex(layer->lslices, valid_polys);
                };

                // Apply to layers N-1 and N-2
                int layer_idx_N1 = (int)i - 1;
                if (layer_idx_N1 >= 0) integrate_polys(m_layers[layer_idx_N1], bridges_vertical);

                int layer_idx_N2 = (int)i - 2;
                if (layer_idx_N2 >= 0) integrate_polys(m_layers[layer_idx_N2], bridges_horizontal);
            }
        }
    }
}

} // namespace Slic3r