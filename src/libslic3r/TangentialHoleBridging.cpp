//ORCA: Tangential sacrificial bridging for counterbore holes
#include "TangentialHoleBridging.hpp"
#include "Print.hpp"
#include "Layer.hpp"
#include "ClipperUtils.hpp"

namespace Slic3r {

void TangentialHoleBridging::apply(PrintObject* object)
{
    // check if ANY region has chbTangential enabled
    bool has_tangential = false;
    for (size_t region_id = 0; region_id < object->num_printing_regions(); ++region_id) {
        if (object->printing_region(region_id).config().counterbore_hole_bridging == chbTangential) {
            has_tangential = true;
            break;
        }
    }
    if (!has_tangential || object->layer_count() < 3)
        return;

    const double nozzle_diameter = object->print()->config().nozzle_diameter.get_at(0);
    const coord_t strut_w = scale_(nozzle_diameter * 2.0);
    const coord_t margin = scale_(nozzle_diameter * 2.0); // anchor margin

    for (size_t i = 2; i < object->layer_count(); ++i) {
        Layer* layer_n = object->get_layer(i);
        Layer* layer_n1 = object->get_layer(i - 1);
        Layer* layer_n2 = object->get_layer(i - 2);

        for (size_t region_id = 0; region_id < object->num_printing_regions(); ++region_id) {
            if (object->printing_region(region_id).config().counterbore_hole_bridging != chbTangential) {
                continue;
            }

            LayerRegion* region_n = layer_n->get_region(region_id);
            if (!region_n || region_n->slices.empty()) continue;

            // Find unsupported areas specifically for this region
            ExPolygons unsupported = diff_ex(region_n->slices.surfaces, layer_n1->lslices, ApplySafetyOffset::Yes);
            if (unsupported.empty()) continue;

            for (const ExPolygon& poly_unsupp : unsupported) {
                // A counterbore overhang must have a hole in the middle (it's a ring or a bridge with holes)
                if (poly_unsupp.holes.empty()) {
                    continue; 
                }

                BoundingBox bbox = get_extents(poly_unsupp);

                // We must ensure the struts will anchor to something in N-1 and N-2.
                // ORCA: Expand contour to ensure anchoring into the solid walls
                Polygons contour_bigger = offset(poly_unsupp.contour, margin);
                Polygons anchors = intersection(contour_bigger, to_polygons(layer_n1->lslices));
                
                if (anchors.empty()) {
                    continue;
                }

                Polygons raw_struts_n1;
                Polygons raw_struts_n2;

                // ORCA: Process ALL holes instead of just the largest one
                // This allows bridging areas with multiple holes to be supported properly.
                for (const Polygon& hole : poly_unsupp.holes) {
                    BoundingBox inner_bbox = get_extents(hole);

                    // ORCA: Check the distance from the hole to the bounding box of the unsupported area.
                    // If the distance is small (e.g. <= 8mm), it's a counterbore lip, so we fill the entire space
                    // to the contour to make a solid block (user preferred behavior).
                    // If the distance is large (e.g. a huge bridge with a small hole), we limit the strut width
                    // to 'strut_w' to avoid filling the entire bridge with massive blocks.
                    const coord_t max_lip = scale_(8.0);
                    
                    coord_t dist_left   = inner_bbox.min.x() - bbox.min.x();
                    coord_t dist_right  = bbox.max.x() - inner_bbox.max.x();
                    coord_t dist_bottom = inner_bbox.min.y() - bbox.min.y();
                    coord_t dist_top    = bbox.max.y() - inner_bbox.max.y();

                    coord_t left_x   = (dist_left <= max_lip)   ? (bbox.min.x() - margin) : (inner_bbox.min.x() - strut_w);
                    coord_t right_x  = (dist_right <= max_lip)  ? (bbox.max.x() + margin) : (inner_bbox.max.x() + strut_w);
                    coord_t bottom_y = (dist_bottom <= max_lip) ? (bbox.min.y() - margin) : (inner_bbox.min.y() - strut_w);
                    coord_t top_y    = (dist_top <= max_lip)    ? (bbox.max.y() + margin) : (inner_bbox.max.y() + strut_w);

                    // Strut Left (Y direction) - for N-2
                    Polygon strut_left;
                    strut_left.points = {
                        Point(left_x,             bbox.min.y() - margin),
                        Point(inner_bbox.min.x(), bbox.min.y() - margin),
                        Point(inner_bbox.min.x(), bbox.max.y() + margin),
                        Point(left_x,             bbox.max.y() + margin)
                    };
                    
                    // Strut Right (Y direction) - for N-2
                    Polygon strut_right;
                    strut_right.points = {
                        Point(inner_bbox.max.x(), bbox.min.y() - margin),
                        Point(right_x,            bbox.min.y() - margin),
                        Point(right_x,            bbox.max.y() + margin),
                        Point(inner_bbox.max.x(), bbox.max.y() + margin)
                    };

                    // Strut Bottom (X direction) - for N-1
                    Polygon strut_bottom;
                    strut_bottom.points = {
                        Point(bbox.min.x() - margin, bottom_y),
                        Point(bbox.max.x() + margin, bottom_y),
                        Point(bbox.max.x() + margin, inner_bbox.min.y()),
                        Point(bbox.min.x() - margin, inner_bbox.min.y())
                    };

                    // Strut Top (X direction) - for N-1
                    Polygon strut_top;
                    strut_top.points = {
                        Point(bbox.min.x() - margin, inner_bbox.max.y()),
                        Point(bbox.max.x() + margin, inner_bbox.max.y()),
                        Point(bbox.max.x() + margin, top_y),
                        Point(bbox.min.x() - margin, top_y)
                    };

                    raw_struts_n2.push_back(strut_left);
                    raw_struts_n2.push_back(strut_right);
                    raw_struts_n1.push_back(strut_bottom);
                    raw_struts_n1.push_back(strut_top);
                }

                // ORCA: Intersect struts with the unsupported area to prevent them from shooting into mid-air
                // on L-shaped or diagonal bridges, guaranteeing they find an anchor in the surrounding wall.
                Polygons final_struts_n1 = intersection(raw_struts_n1, contour_bigger);
                Polygons final_struts_n2 = intersection(raw_struts_n2, contour_bigger);

                // ORCA: Subtract all holes of this unsupported area from the generated struts
                // so we don't accidentally cover other holes in a multi-hole bridge.
                Polygons all_holes = poly_unsupp.holes;
                final_struts_n1 = diff(final_struts_n1, all_holes);
                final_struts_n2 = diff(final_struts_n2, all_holes);

                if (final_struts_n1.empty() && final_struts_n2.empty()) continue;

                // Add to lslices (overall layer boundary)
                layer_n2->lslices = union_ex(layer_n2->lslices, final_struts_n2);
                layer_n1->lslices = union_ex(layer_n1->lslices, final_struts_n1);
                
                // --- INTEGRATE INTO LAYER N-1 ---
                if (!layer_n1->regions().empty() && !final_struts_n1.empty()) {
                    // Subtract struts from all OTHER regions to prevent multicolor overlap
                    for (size_t r = 0; r < layer_n1->regions().size(); ++r) {
                        if (r == region_id) continue;
                        LayerRegion* other_region = layer_n1->get_region(r);
                        if (!other_region || other_region->slices.empty()) continue;
                        ExPolygons cut_polys = diff_ex(other_region->slices.surfaces, final_struts_n1);
                        other_region->slices.set(cut_polys, stInternal);
                    }

                    // Add struts specifically to the correct region
                    LayerRegion* target_region = layer_n1->get_region(region_id);
                    if (target_region) {
                        Polygons polys_1 = to_polygons(target_region->slices.surfaces);
                        for (const Polygon& p : final_struts_n1) polys_1.push_back(p);
                        target_region->slices.set(union_ex(polys_1), stInternal);
                    }
                }
                
                // --- INTEGRATE INTO LAYER N-2 ---
                if (!layer_n2->regions().empty() && !final_struts_n2.empty()) {
                    // Subtract struts from all OTHER regions to prevent multicolor overlap
                    for (size_t r = 0; r < layer_n2->regions().size(); ++r) {
                        if (r == region_id) continue;
                        LayerRegion* other_region = layer_n2->get_region(r);
                        if (!other_region || other_region->slices.empty()) continue;
                        ExPolygons cut_polys = diff_ex(other_region->slices.surfaces, final_struts_n2);
                        other_region->slices.set(cut_polys, stInternal);
                    }

                    // Add struts specifically to the correct region
                    LayerRegion* target_region = layer_n2->get_region(region_id);
                    if (target_region) {
                        Polygons polys_2 = to_polygons(target_region->slices.surfaces);
                        for (const Polygon& p : final_struts_n2) polys_2.push_back(p);
                        target_region->slices.set(union_ex(polys_2), stInternal);
                    }
                }
            }
        }
    }
}

} // namespace Slic3r
