import math
import os

from build123d import *
from build123d import export_stl

from droplet import Droplet

# To build the final 3D model, enter the environment and run the following command:
# $ final=true python -m trace --ignore-dir=$(python -c 'import sys ; print(":".join(sys.path)[1:])') --trace main.py

# %%

# General parameters
tol = 0.2 * MM  # Tolerance for operations
eps = 0.0001 * MM  # Epsilon for operations
min_wall = 0.4 * MM  # Minimum wall thickness on XY (common for FDM 3D printing)
wall = 3 * min_wall  # 3 perimeters on XY

# Bottle parameters
bottle_body_height = 215 * MM
bottle_body_radius = 8.2 / 2 * CM
bottle_body_fillet = 1 * CM
bottle_top_extrusion = 16 * MM
bottle_top_angle = 24  # degrees

# Bike parameters
bike_screw_head_radius = 6 * MM
bike_screw_radius = 3 * MM
bike_screw_head_height = 5 * MM
bike_screw_separation = 64 * MM
bike_screw_separation_tolerance = 10 * MM
bike_bottom_space = 75 * MM

# Holder parameters
holder_thickness = 2 * wall
holder_core_pct = 0.75  # > 0, <= 0.9
holder_side_grabber_side = 20 * MM
holder_side_grabber_max_angle = 145  # degrees
holder_side_grabber_min_angle_offset_top = -25  # degrees (also applied to 2nd, 3rd... inner grabbers)
holder_side_grabber_min_angle_offset_bottom = 0  # degrees
holder_side_grabber_smooth = 1.25  # >= 1
holder_side_grabber_count = 3  # >= 0 (slow operation, so disable if not needed while designing)
holder_side_grabber_max_angle_loss = 35  # degrees
holder_side_grabber_z_loss = holder_side_grabber_side * 1.5  # degrees
holder_bottom_grabber_min_angle = 90  # degrees
holder_bottom_grabber_side = 18 * MM
holder_bottom_grabber_side_center = holder_bottom_grabber_side * 0.65  # HACK: Helps stabilize the multisection sweep
holder_bottom_grabber_num_section_samples = [10, 8, 4]  # Can cause lots of issues...
holder_bottom_grabber_max_z = bottle_body_height / 2 - holder_bottom_grabber_side / 2
holder_bottom_grabber_smooth = 1.25  # >= 1
holder_bottom_grabber_count = 3  # >= 0 (slow operation, so disable if not needed while designing)
holder_bottom_grabber_min_angle_loss = 35  # degrees
holder_bottom_grabber_z_loss = holder_bottom_grabber_side * 1.5
holder_bottom_reinforcement = 3 * MM  # + holder_thickness
holder_zip_tie_hole_radius = 2.5 * MM  # <= holder_thickness + bike_screw_radius * 2

with BuildPart() as bottle:
    with BuildSketch():
        Circle(radius=bottle_body_radius)
    extrude(amount=bottle_body_height, taper=0)
    extrude(faces().group_by(Axis.Z)[-1], amount=bottle_top_extrusion, taper=bottle_top_angle)
    fillet(edges(), radius=bottle_body_fillet)
bottle_height = bottle_body_height + bottle_top_extrusion

core_plane = Plane.XY.shift_origin((bottle_body_radius + tol, 0)).offset(-holder_thickness)
with BuildPart() as holder_core:
    # Extrude the main profile that connects the holder and the tube
    with BuildSketch(core_plane):  # holder_core_profile
        with Locations((bike_screw_head_height + holder_thickness, 0)):
            full_profile = bottle_body_radius * 2
            used_profile_y = full_profile * holder_core_pct
            holder_core_angle = math.asin((used_profile_y / 2) / (full_profile / 2)) * 180 / math.pi
            used_profile_x = full_profile / 2
            Rectangle(used_profile_x + tol + holder_thickness, used_profile_y, align=(Align.MAX, Align.CENTER))
        Circle(radius=bottle_body_radius + tol, align=(Align.MAX, Align.CENTER), mode=Mode.SUBTRACT)
        # TODO: Profile for the tube...
    fillet_top_radius = (bike_screw_head_height + holder_thickness - tol) / 2  # Used at the end
    extrude(amount=bottle_height + holder_thickness)

    # Base for the bottle
    Cylinder(height=holder_thickness, radius=bottle_body_radius + holder_thickness + 2 * tol,
             align=(Align.CENTER, Align.CENTER, Align.MAX))
    with BuildSketch():  # Make a hull for a stronger bottom
        add(holder_core.faces().group_by(Axis.Z)[0])
        make_hull()
    extrude(amount=-holder_thickness)

    # Grab the top of the bottle
    with BuildSketch(
            Plane(faces().group_by(Axis.Z)[-1].edges().filter_by(GeomType.CIRCLE).edge() @ 0.5, (1, 0, 0), (0, 1, 0))):
        off = (bottle_top_extrusion - fillet_top_radius)
        extrude_by = math.tan(math.radians(bottle_top_angle)) * off  # Half to be a ramp on both sides
        Rectangle(extrude_by, bottle_top_extrusion, align=(Align.MAX, Align.MIN))
        chamfer(edges().group_by(Axis.X)[0].vertices().group_by(Axis.Y)[-1], off - eps, extrude_by - eps)
    extrude(amount=used_profile_y / 2, both=True)

    # Add a zip tie hole in the middle of the left face, through the complete holder
    with BuildSketch(core_plane.offset(bottle_height / 2).location * Plane.XZ.location):
        with Locations(((bike_screw_head_height + holder_thickness) / 2, 0)):
            Droplet(radius=holder_zip_tie_hole_radius, roof_angle=45)
    extrude(amount=9999, both=True, mode=Mode.SUBTRACT)


    def screw_holes(rad: float) -> Sketch:
        with BuildSketch(core_plane.location * Plane.YZ.offset(-holder_thickness).location) as sk:
            with Locations((0, bike_bottom_space), (0, bike_bottom_space + bike_screw_separation)):
                Rectangle(2 * rad, bike_screw_separation_tolerance)
                with Locations((0, bike_screw_separation_tolerance / 2), (0, -bike_screw_separation_tolerance / 2)):
                    Droplet(radius=rad, roof_angle=45, align=None)
        return sk.sketch


    # Add screw holes
    extrude(screw_holes(bike_screw_head_radius), amount=bike_screw_head_height + holder_thickness, mode=Mode.SUBTRACT)
    extrude(screw_holes(bike_screw_radius), amount=9999, mode=Mode.SUBTRACT)


# Now, for the hard part, design the grabber to be 3D printable wrapping around the bottle
def grabber_profile_line_fn(side: float, rad: float = bottle_body_radius + tol) -> (Curve, Vector):
    with BuildLine() as _grabber_profile_line:
        arc_in = RadiusArc((-side / 2, holder_thickness / 2),
                           (side / 2, holder_thickness / 2), rad)
        arc_out = RadiusArc((-side / 2, -holder_thickness / 2),
                            (side / 2, -holder_thickness / 2),
                            rad + holder_thickness)
        # Offset center to avoid slighltly overlapping bottle, which tends to cause issues
        _to_center = (arc_out @ 0.5 + arc_in @ 0.5) / 2 + Vector(0, -tol, 0)
        Spline(arc_in @ 1, arc_out @ 1, tangents=[arc_in % 1, -(arc_out % 1)], tangent_scalars=[2.5, 2.5])
        Spline(arc_out @ 0, arc_in @ 0, tangents=[-(arc_out % 0), arc_in % 0], tangent_scalars=[2.5, 2.5])
        del arc_in, arc_out
    return _grabber_profile_line.line, _to_center


with BuildPart() as grabber:
    # Build side grabbers
    for hsg_index in range(holder_side_grabber_count):
        bottom_angle_offset = holder_side_grabber_min_angle_offset_bottom if hsg_index == 0 \
            else holder_side_grabber_min_angle_offset_top
        grabber_lines = []
        for bi_normal_off in [eps, 0]:  # Build the binormal at the same time :)
            with BuildLine() as grabber_line:  # 3D line to be used as a reference
                xyz = []
                num_samples = int(bottle_height)  # ~1 sample per mm
                for z_index in range(num_samples + 1):
                    cur_holder_z_offset = holder_side_grabber_z_loss * hsg_index
                    z = z_index / num_samples * (bottle_height - cur_holder_z_offset * 2) + cur_holder_z_offset
                    max_angle = holder_side_grabber_max_angle - holder_side_grabber_max_angle_loss * hsg_index
                    min_angle = holder_core_angle + \
                                (1 - z_index / num_samples) * bottom_angle_offset + \
                                (z_index / num_samples) * holder_side_grabber_min_angle_offset_top
                    angle = max_angle - math.fabs(z_index / (num_samples / 2) - 1) ** holder_side_grabber_smooth * \
                            (max_angle - min_angle) + bi_normal_off
                    x = math.cos(math.radians(angle)) * (bottle_body_radius + tol + holder_thickness / 2)
                    y = math.sin(math.radians(angle)) * (bottle_body_radius + tol + holder_thickness / 2)
                    xyz.append((x, y, z))
                Spline(*xyz)
            grabber_lines.append(grabber_line.line)
            del grabber_line

        grabber_profile_line, to_center = grabber_profile_line_fn(side=holder_side_grabber_side)
        with BuildSketch(Location(grabber_lines[0] @ 0, (0, 0, holder_core_angle + bottom_angle_offset - 90)) *
                         Location(-to_center)) as grabber_profile:
            make_face(grabber_profile_line.edges())
        del grabber_profile_line  # Also used for the bottom grabber

        sweep(grabber_profile.sketch, grabber_lines[0], binormal=grabber_lines[1].edge())
        del grabber_lines, grabber_profile

    if holder_side_grabber_count > 0:
        mirror()

    # Build the bottom grabber
    for hbg_index in range(holder_bottom_grabber_count):
        grabber_lines = []
        min_angle = holder_bottom_grabber_min_angle + holder_bottom_grabber_min_angle_loss * hbg_index
        for bi_normal_off in [eps, 0]:  # Build the binormal at the same time :)
            with BuildLine() as grabber_line:  # 3D line to be used as a reference
                xyz = []
                num_samples = int(bottle_height)  # ~1 sample per mm
                for z_index in range(num_samples + 1):
                    angle = z_index / num_samples * (360 - 2 * min_angle) + min_angle + \
                            bi_normal_off * (z_index / (num_samples / 2) - 1)
                    max_z = holder_bottom_grabber_max_z - holder_bottom_grabber_z_loss * hbg_index
                    z = max_z - math.fabs(z_index / (num_samples / 2) - 1) ** holder_bottom_grabber_smooth * max_z \
                        + bi_normal_off * (1 - math.fabs(z_index / (num_samples / 2) - 1))
                    x = math.cos(math.radians(angle)) * (bottle_body_radius + tol + holder_thickness / 2)
                    y = math.sin(math.radians(angle)) * (bottle_body_radius + tol + holder_thickness / 2)
                    xyz.append(Vector(x, y, z))
                Spline(*xyz)
            grabber_lines.append(grabber_line.line)
            del grabber_line

        sweep_sections = []
        num_samples = holder_bottom_grabber_num_section_samples[hbg_index]
        assert num_samples % 2 == 0, 'num_samples must be even'
        for i in range(num_samples + 1):
            pct = i / num_samples
            flip = 1 if i < num_samples else -1
            z_dir = grabber_lines[1] % pct
            if i == 0 or i == num_samples:
                z_dir = Vector(0, 0, 1)
            binormal = (grabber_lines[0] @ pct - grabber_lines[1] @ pct).normalized()
            plane = Plane(grabber_lines[1] @ pct, binormal * flip, z_dir)
            roundness = (math.fabs(pct - 0.5) * 2) ** holder_bottom_grabber_smooth  # Made up formula :)
            size = (math.fabs(pct - 0.5) * 2) * holder_bottom_grabber_side + \
                   (1 - (math.fabs(pct - 0.5) * 2)) * holder_bottom_grabber_side_center
            grabber_profile_line, to_center = grabber_profile_line_fn(
                side=size, rad=(bottle_body_radius + tol) / (roundness + eps))
            with BuildSketch(plane.location * Location(-to_center)) as grabber_profile:
                make_face(grabber_profile_line.edges())
            sweep_sections.append(grabber_profile.sketch)
            del grabber_profile_line, grabber_profile

        sweep(sweep_sections, grabber_lines[1], binormal=grabber_lines[0].edge(), multisection=True)
        del grabber_lines, sweep_sections

    if holder_side_grabber_count + holder_bottom_grabber_count == 0:
        add(holder_core)  # HACK to avoid crashing while designing


def bb_to_box(_bb: BoundBox) -> Box:
    return Box(_bb.max.X - _bb.min.X, _bb.max.Y - _bb.min.Y, _bb.max.Z - _bb.min.Z).translate(_bb.center())


# Final merge
bike_bottle_holder = holder_core.part + grabber.part
# - Cleaning up the merge (the grabber overflows a bit)
bb = holder_core.part.bounding_box()  # Clean up the top
bb.max.X, bb.min.X, bb.max.Y, bb.min.Y = 9999, -9999, 9999, -9999
bike_bottle_holder = bike_bottle_holder & bb_to_box(bb)

# Reinforce the bottom
to_reinforce = bike_bottle_holder.faces().filter_by(Plane.XY).group_by(Axis.Z)[0]  # Add bottom reinforcement
bike_bottle_holder += extrude(to_reinforce, amount=holder_bottom_reinforcement, taper=45)
to_fillet = bike_bottle_holder.edges().filter_by(Plane.XY).group_by(Axis.Z)[1]
bike_bottle_holder = fillet(to_fillet, radius=holder_thickness)

# Final fillets / chamfers
to_fillet = bike_bottle_holder.edges().filter_by(GeomType.LINE).group_by(SortBy.LENGTH)[-1].group_by(Axis.X)[-1]
bike_bottle_holder = fillet(to_fillet, radius=holder_thickness)
to_fillet = bike_bottle_holder.edges().filter_by(GeomType.LINE) \
    .filter_by(Axis.Z).group_by(Axis.X)[-1].group_by(Axis.Z)[0]
bike_bottle_holder = fillet(to_fillet, radius=holder_thickness)
to_fillet = bike_bottle_holder.edges().filter_by(GeomType.LINE).filter_by(Plane.XY).group_by(Axis.Z)[-1]
bike_bottle_holder = fillet(to_fillet, radius=fillet_top_radius)
del to_reinforce, holder_core, grabber, to_fillet  # , bottle

if os.getenv('final'):
    export_stl(bike_bottle_holder, 'bike-bottle-holder.stl')
else:
    from yacv_server import *

    show_all()

    if os.getenv('yacv_export'):
        export_all('.', lambda name, obj: name == 'bike_bottle_holder')
