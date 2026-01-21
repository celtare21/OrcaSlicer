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
            if (ov.holes.empty()) continue;

            // Find the largest inner hole (D1) - assume it's the center hole we want to keep open
            const Polygon* hole_D1 = &ov.holes[0];
            double max_area = 0;
            for (const Polygon& h : ov.holes) {
                double area = std::abs(h.area());
                if (area > max_area) {
                    max_area = area;
                    hole_D1 = &h;
                }
            }

            BoundingBox hole_bbox = hole_D1->bounding_box();
            Point center = hole_bbox.center();
            double r_small = (double)std::min(hole_bbox.size().x(), hole_bbox.size().y()) / 2.0;

            // 1. Create "Bridge Base" = Overhang Region + Anchor (3mm into wall)
            // We expand the outer contour of the overhang to anchor it, but keep the inner hole.
            
            double anchor = scale_(3.0);
            Polygons ov_contours;
            ov_contours.push_back(ov.contour);
            ExPolygons expanded_contours = offset_ex(ov_contours, anchor);
            
            // Re-subtract the original hole to ensure we don't block the center
            Polygons holes;
            holes.push_back(*hole_D1);
            ExPolygons bridge_base = diff_ex(expanded_contours, holes);

            // 2. Define Masks for Tangential Bridges
            // Vertical Mask: Covers Left/Right sides (|x| > r_small)
            // Horizontal Mask: Covers Top/Bottom sides (|y| > r_small)
            
            double mask_size = scale_(1000.0); // Large enough to cover the part
            
            auto create_mask = [&](bool vertical) {
                ExPolygons mask;
                for (int sign : {-1, 1}) {
                    Polygon p;
                    p.points.resize(4);
                    // If vertical: x from [r, huge], y from [-huge, huge]
                    // If horizontal: y from [r, huge], x from [-huge, huge]
                    
                    coord_t r_start = (sign == 1) ? static_cast<coord_t>(r_small) : -static_cast<coord_t>(mask_size);
                    coord_t r_end   = (sign == 1) ? static_cast<coord_t>(mask_size) : -static_cast<coord_t>(r_small);
                    
                    if (vertical) {
                        p.points[0] = center + Point(r_start, -static_cast<coord_t>(mask_size));
                        p.points[1] = center + Point(r_end,   -static_cast<coord_t>(mask_size));
                        p.points[2] = center + Point(r_end,    static_cast<coord_t>(mask_size));
                        p.points[3] = center + Point(r_start,  static_cast<coord_t>(mask_size));
                    } else {
                        p.points[0] = center + Point(-static_cast<coord_t>(mask_size), r_start);
                        p.points[1] = center + Point( static_cast<coord_t>(mask_size), r_start);
                        p.points[2] = center + Point( static_cast<coord_t>(mask_size), r_end);
                        p.points[3] = center + Point(-static_cast<coord_t>(mask_size), r_end);
                    }
                    mask.push_back(ExPolygon(p));
                }
                return mask;
            };

            ExPolygons mask_vertical = create_mask(true);
            ExPolygons mask_horizontal = create_mask(false);

            auto integrate_polys = [&](Layer* layer, const ExPolygons& polys_to_add) {
                if (layer->region_count() == 0) return;
                
                // 1. Clip to Footprint (Outer hull of the object)
                // Use union of contours to get the filled shape of the layer.
                Polygons layer_outlines;
                for (const ExPolygon& exp : layer->lslices) {
                    layer_outlines.push_back(exp.contour);
                }
                ExPolygons footprint = union_ex(layer_outlines);
                
                ExPolygons valid_polys = intersection_ex(polys_to_add, footprint);
                
                // 2. Merge into Region 0
                LayerRegion* region = layer->get_region(0);
                ExPolygons existing_polys = to_expolygons(region->slices.surfaces);
                ExPolygons merged_polys = union_ex(existing_polys, valid_polys);
                
                region->slices.surfaces.clear();
                region->slices.append(merged_polys, stInternal);
                
                // 3. Update global lslices
                layer->lslices = union_ex(layer->lslices, valid_polys);
            };

            // Apply to layers N-1 and N-2
            // N-1 (Index i-1): Vertical Bridge (Supports Left/Right of ring)
            int layer_idx_N1 = (int)i - 1;
            if (layer_idx_N1 >= 0) {
                ExPolygons bridge_N1 = intersection_ex(bridge_base, mask_vertical);
                integrate_polys(m_layers[layer_idx_N1], bridge_N1);
            }

            // N-2 (Index i-2): Horizontal Bridge (Supports Top/Bottom of ring)
            int layer_idx_N2 = (int)i - 2;
            if (layer_idx_N2 >= 0) {
                ExPolygons bridge_N2 = intersection_ex(bridge_base, mask_horizontal);
                integrate_polys(m_layers[layer_idx_N2], bridge_N2);
            }
        }
    }
}

} // namespace Slic3r
