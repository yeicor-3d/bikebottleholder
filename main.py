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

# Bottle parameters (should be a perfect fit)
bottle_body_height = 215 * MM
bottle_body_radius = 8.18 / 2 * CM
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
holder_zip_tie_hole_radius = 2.5 * MM  # <= holder_thickness + bike_screw_radius * 2
holder_bottom_reinforcement = 3 * MM
holder_grabber_side = 20 * MM  # Approximately...
holder_grabber_angle = 42  # degrees (of deviation from going straight down)
holder_grabber_sep = 10 * MM  # Approximately...


def bb_to_box(_bb: BoundBox) -> Box:
    return Box(_bb.max.X - _bb.min.X, _bb.max.Y - _bb.min.Y, _bb.max.Z - _bb.min.Z,
               mode=Mode.PRIVATE).translate(_bb.center())

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
            full_profile = (bottle_body_radius + tol) * 2
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
    top_face_inner_edge = faces().group_by(Axis.Z)[-1].edges().filter_by(GeomType.CIRCLE).edge()
    with BuildPart():
        with BuildSketch(Plane(top_face_inner_edge @ 0.5, (1, 0, 0), (0, 1, 0))):
            with BuildLine():
                extrude_by = math.tan(math.radians(bottle_top_angle)) * bottle_top_extrusion
                Polyline((0, 0), (0, bottle_top_extrusion), (-extrude_by, 0), close=True)
            make_face()
        del top_face_inner_edge
        # extrude(amount=used_profile_y / 2, both=True)
        revolve(revolution_arc=holder_core_angle)
        # Perform a second revolve to align with border (simpler fillet later)
        rev_axis = Axis(vertices().group_by(Axis.Y)[-1].group_by(Axis.Z)[-1].vertex().center(), (0, 0, -1))
        revolve(faces().group_by(Axis.Y)[-1], axis=rev_axis, revolution_arc=holder_core_angle)
        mirror()

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
grabbers = []  # Boolean operations on sweeps may fail, so keep them separate
vertical_side = holder_grabber_side / math.tan(math.radians(holder_grabber_angle))
z_start_top = bottle_height - fillet_top_radius
holder_grabber_count = int(z_start_top // (vertical_side + holder_grabber_sep))
# holder_grabber_count = 0  # For performance while testing
for hsg_index in range(holder_grabber_count):
    grabber_lines = []
    z_start = z_start_top - hsg_index * (vertical_side + holder_grabber_sep)
    num_samples = int(z_start)  # ~1mm per sample (should be way more than enough as we are using splines)
    z_per_step = (1 / (num_samples - 1)) * z_start
    side_per_step = z_per_step * math.tan(math.radians(holder_grabber_angle))
    start_angle = holder_core_angle / 2  # Clearly inside the core (to avoid overflowing corners at the top
    angle_per_step = math.degrees(math.asin(side_per_step / (bottle_body_radius + tol + holder_thickness / 2)))
    print('Grabber', hsg_index + 1, '/', holder_grabber_count)
    for bi_normal_off in [eps, 0]:  # Build the binormal at the same time :)
        with BuildLine() as grabber_line:  # 3D sweep path
            xyz = []
            for z_index in range(num_samples):
                z = z_start - z_index * z_per_step
                angle = start_angle + z_index * angle_per_step + bi_normal_off  # Always horizontal
                x = math.cos(math.radians(angle)) * (bottle_body_radius + tol + holder_thickness / 2)
                y = math.sin(math.radians(angle)) * (bottle_body_radius + tol + holder_thickness / 2)
                xyz.append((x, y, z))
            Spline(*xyz)
        grabber_lines.append(grabber_line.line)
        del grabber_line

    with BuildSketch(Location(grabber_lines[0] @ 0, start_angle - 90)) as grabber_profile:
        with BuildLine() as grabber_profile_line:
            arc_in = RadiusArc((-holder_grabber_side / 2, holder_thickness / 2),
                               (holder_grabber_side / 2, holder_thickness / 2), bottle_body_radius + tol)
            arc_out = RadiusArc((-holder_grabber_side / 2, -holder_thickness / 2),
                                (holder_grabber_side / 2, -holder_thickness / 2),
                                bottle_body_radius + tol + holder_thickness)
            # Offset center to avoid slightly overlapping bottle, which tends to cause issues
            _wanted_center = (arc_out @ 0.5 + arc_in @ 0.5) / 2 + Vector(0, -tol, 0)
            Spline(arc_in @ 1, arc_out @ 1, tangents=[arc_in % 1, -(arc_out % 1)], tangent_scalars=[2.5, 2.5])
            Spline(arc_out @ 0, arc_in @ 0, tangents=[-(arc_out % 0), arc_in % 0], tangent_scalars=[2.5, 2.5])
            del arc_in, arc_out
        make_face(grabber_profile_line.line.move(Location(-_wanted_center)).edges())
    del grabber_profile_line  # Also used for the bottom grabber

    sweep_obj = sweep(grabber_profile.sketch, grabber_lines[0], binormal=grabber_lines[1].edge(), mode=Mode.PRIVATE)
    del grabber_lines, grabber_profile

    print('Mirroring...')
    grabbers.append(sweep_obj)
    grabbers.append(mirror(sweep_obj, mode=Mode.PRIVATE))
    del sweep_obj

# Final merge
bike_bottle_holder = holder_core.part  # + grabber.part

# Reinforce the bottom
to_reinforce = bike_bottle_holder.faces().filter_by(Plane.XY).group_by(Axis.Z)[0]  # Add bottom reinforcement
bike_bottle_holder += extrude(to_reinforce, amount=holder_bottom_reinforcement, taper=45)
to_fillet = bike_bottle_holder.edges().filter_by(Plane.XY).group_by(Axis.Z)[1]
bike_bottle_holder = fillet(to_fillet, radius=holder_thickness)  # 1

# Final fillets / chamfers
to_fillet = bike_bottle_holder.edges().filter_by(GeomType.LINE).group_by(SortBy.LENGTH)[-1].group_by(Axis.X)[-1]
bike_bottle_holder = fillet(to_fillet, radius=holder_thickness)  # 2
to_fillet = bike_bottle_holder.edges().filter_by(GeomType.LINE) \
    .filter_by(Axis.Z).group_by(Axis.X)[-1].group_by(Axis.Z)[0]
bike_bottle_holder = fillet(to_fillet, radius=holder_thickness)  # 3
to_fillet = bike_bottle_holder.edges().group_by(Axis.Z)[-1]
bike_bottle_holder = fillet(to_fillet, radius=holder_thickness)  # 4
del holder_core, to_reinforce, to_fillet  # , bottle

# Assemble the grabbers, as fusing them is too slow/buggy and the slicer should be able to handle it
bike_bottle_holder = Compound([bike_bottle_holder] + grabbers)
del grabbers

if os.getenv('export_stl'):
    print('Exporting STL file...')
    export_stl(bike_bottle_holder, 'bike-bottle-holder.stl')

try:
    from yacv_server import *

    show_all()

    if os.getenv('export_yacv'):
        print('Exporting YACV file...')
        export_all('.', lambda name, obj: name == 'bike_bottle_holder')
except BaseException as e:
    print(f'yacv_server not found or another error happened, skipping visualization: {e}')
