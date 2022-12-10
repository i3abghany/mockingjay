from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import PrivateL1PrivateL2CacheHierarchy
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.resources.resource import Resource, CustomResource
from gem5.simulate.simulator import Simulator

from m5.SimObject import *
from m5.params import *
from m5.objects.ReplacementPolicies import *

class CacheHier(PrivateL1PrivateL2CacheHierarchy):
    def __init__(self, l1d_size="1kB", l1i_size="1kB", l2_size="64kB"):
        super().__init__(l1d_size, l1i_size, l2_size)

# Obtain the components.
cache_hierarchy = CacheHier(l1d_size="1kB", l1i_size="1kB", l2_size="64kB")
memory = SingleChannelDDR3_1600("1GiB")
processor = SimpleProcessor(cpu_type=CPUTypes.ATOMIC, num_cores=1)

#Add them to the board.
board = SimpleBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Set the workload.
# binary = Resource("x86-hello64-static")
# binary = CustomResource("bench.X86")
binary = CustomResource("bench.X86")
board.set_se_binary_workload(binary)

# Setup the Simulator and run the simulation.
simulator = Simulator(board=board)
simulator.run()
