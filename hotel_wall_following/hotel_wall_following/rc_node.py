"""Optional RC wall source running on the PC, upstream of mux and AEB."""

from hotel_wall_following.rc_core import wall_command, WallConfig
from hotel_wall_following.rc_support import ScanCommandSource, spin_source


class WallNode(ScanCommandSource):
    """Use the validated wall algorithm with a shared expiring ROS source."""

    def __init__(self, **kwargs):
        """Start disabled until explicit enable heartbeats arrive."""
        super().__init__('wall', WallConfig, wall_command, **kwargs)


def main(args=None):
    """Run the optional PC behavior without opening hardware."""
    spin_source(WallNode, args)
