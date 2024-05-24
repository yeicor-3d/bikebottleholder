from typing import Union

from build123d import *
from build123d import export_stl


class Droplet(BaseSketchObject):
    """A circle with a roof on top, useful for 3D printing with no support material"""

    def __init__(
            self,
            radius: float,
            roof_angle: float = 45,
            rotation: float = 0,
            align: Union[Align, tuple[Align, Align], None] = None,
            mode: Mode = Mode.ADD,
    ):
        assert 0 < roof_angle < 90, "Roof angle must be between 0 and 90 degrees"
        with BuildSketch() as droplet:
            with BuildLine():
                arc = CenterArc(center=(0, 0), radius=radius, start_angle=90 + roof_angle,
                                arc_size=360 - 2 * roof_angle)
                center_line = Line((0, 0), (0, 99999999), mode=Mode.PRIVATE)
                # Line(arc @ 0, arc @ 1)
                IntersectingLine(arc @ 0, -(arc % 0), center_line)
                IntersectingLine(arc @ 1, arc % 1, center_line)
            make_face()
        super().__init__(obj=droplet.sketch, rotation=rotation, align=align, mode=mode)


if __name__ == "__main__":
    from yacv_server import show_all

    droplet = Droplet(radius=10, roof_angle=60)

    show_all()
    export_stl(droplet, 'droplet.stl')
