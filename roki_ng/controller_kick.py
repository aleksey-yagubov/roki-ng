"""Read-only execution witness for the original IronArt roki2met project.

Not a generic slot-number query: Zubr's Kondo RAM address 2 is a constant.
roki2Kick has framestep=2 (local 18), rotationYieldRight=.23 (local 16).
On script restart the VM clears local RAM. mixingRoki2met then sets locals
2 and 3 to 100; locals 16/18 remain zero. Require both states in order.
The map comes from the project's generated roki2met.py, not Kondo byte RAM.
Other controller projects need their own verified execution contract.
"""

import struct

from .wire import Fault

KICK_SLOT = 31
# Kondo motionPlay is one-based; CsCore::scriptStart(CS_SLOT_STOP=65).
STOP_SLOT = 66
KICK_PARAMETERS = ((1029, 'right'), (1028, 'power'), (1030, 'offset'))


class KickWitness:
    # Closing framestep read rejects a snapshot straddling the slot transition.
    ADDRESSES = (18, 16, 2, 3, 18)
    ROTATION_BITS = struct.unpack('<i', struct.pack('<f', .23))[0]

    def __init__(self):
        self.values = []
        self.started = False

    def poll(self, hardware):
        self.values.append(hardware.controller_read(self.ADDRESSES[len(self.values)]))
        if len(self.values) < len(self.ADDRESSES):
            return False
        first, rotation, errors, step, last = self.values
        self.values = []
        if first == last == 2 and rotation == self.ROTATION_BITS:
            self.started = True
            if errors:
                raise Fault('invalid_motion', f'Controller kick IK failures: {errors}')
        return (self.started and first == last == rotation == 0
                and errors == step == 100)
