"""Optional RC gap source running on the PC, upstream of mux and AEB."""

from hotel_ttc_follow_the_gap.rc_core import gap_command, GapConfig
from hotel_wall_following.rc_support import ScanCommandSource, spin_source


class GapNode(ScanCommandSource):
    """Use the validated gap algorithm with a shared expiring ROS source."""

    def __init__(self, **kwargs):
        """Start disabled until explicit enable heartbeats arrive."""
        super().__init__('gap', GapConfig, gap_command, **kwargs)


def main(args=None):
    """Run the optional PC behavior without opening hardware."""
    spin_source(GapNode, args)
