# SPDX-License-Identifier: Apache-2.0
"""Observed behaviour of the pinned CANopenNode slave (`canopend`, node 2), as constants.

Every constant here is pinned by a test in test_canopennode_config.py (named beside it) that
measures it on vcan against the binary built into the dev image. The master interop matrix
(test_master_matrix.py, rows N1-N7) takes its expectations from here and from cia301.py, never
from the master's own code. A fact that is a defect or a limit of the pinned build is stated as
such: the matrix row that depends on it is redesigned around it, not weakened.
"""

# docker/Dockerfile ARGs (test_canopend_build_matches_the_pinned_commits).
CANOPENLINUX_COMMIT = "f1348d4072cdabea4c3435a13c721ac29ab4cc91"  # master, no release after v4.0
CANOPENNODE_COMMIT = "ac2140717c3c498d9b0351bce052bab630a74764"  # tag v4.1, has PR #320 (84487ef)
CANOPENLINUX_SHA256 = "806b1b2e22561a537a1e40df16f1c0aae84d5e0c1128e93bfcde197774bcb640"
CANOPENNODE_SHA256 = "9bba0fe0c8def3d99aa5457e79a429b058e6495271043b5755cbefad4d44ccb7"

NODE_ID = 2

# --- NMT (test_canopend_does_not_self_start, test_canopend_reports_nmt_state) -------------------
# After boot-up the node stays Pre-operational (heartbeat 0x7F) until a master starts it.
# Log lines (stderr, priority prefix <5> NOTICE) that report each NMT state change.
LOG_NMT = {
    "initializing": '<5>CANopen NMT state changed to: "initializing" (0)',
    "pre_operational": '<5>CANopen NMT state changed to: "pre-operational" (127)',
    "operational": '<5>CANopen NMT state changed to: "operational" (5)',
    "stopped": '<5>CANopen NMT state changed to: "stopped" (4)',
}
LOG_RUNNING = "<6>CANopen device, Node ID = 0x02, running ..."
LOG_FINISHED = "<6>CANopen device, Node ID = 0x02, finished"


def log_running(node_id):
    """LOG_RUNNING of a canopend started with `-i node_id` (N7 runs nodes 1-6)."""
    return LOG_RUNNING.replace("0x02", f"0x{node_id:02X}")

# --- object dictionary (test_canopend_example_od_facts) ----------------------------------------
DEVICE_TYPE = 0x00000000  # 0x1000, the shipped example OD value
IDENTITY = {1: 0, 2: 0, 3: 0, 4: 0}  # 0x1018:01..04 (vendor, product, revision, serial), all 0
IDENTITY_HIGHEST_SUB = 4
SHIPPED_HEARTBEAT_MS = 0  # 0x1017 in the shipped OD: no periodic heartbeat
SEEDED_HEARTBEAT_MS = 100  # 0x1017 from the pre-seeded storage (canopend.seed_storage)
SYNC_COB_ID_1005 = 0x80
EMCY_COB_ID_1014 = 0x82  # 0x1014 reads 0x80 + node-ID

# --- SDO aborts (test_canopend_sdo_abort_codes) -------------------------------------------------
ABORT_NO_OBJECT = 0x06020000  # unknown object, upload or download
ABORT_NO_SUBINDEX = 0x06090011  # 0x1018:05
ABORT_READ_ONLY = 0x06010002  # download to 0x1000 or to 0x1018:01
ABORT_LENGTH_TOO_LOW = 0x06070013  # 1 byte into a 2-byte (0x1017) or 4-byte object
ABORT_LENGTH_TOO_HIGH = 0x06070012  # 4 bytes into the 2-byte 0x1017
ABORT_INVALID_VALUE = 0x06090030  # a COB-ID value the SDO-client object (0x1280:1/2) refuses
ABORT_NOT_MAPPABLE = 0x06040041  # a mapping entry for an object that is not mappable or does not fit

# --- PDO configuration (test_canopend_pdo_configuration_facts) ---------------------------------
# Defaults after a (re)start: all four RPDOs and TPDOs disabled (bit 31), no mapping,
# transmission type 0xFE (event driven, manufacturer specific).
DEFAULT_RPDO1_COB = 0x80000202  # 0x1400:01
DEFAULT_TPDO1_COB = 0xC0000182  # 0x1800:01 (bit 30: no RTR)
DEFAULT_PDO_TYPE = 0xFE
# The communication (0x1400/0x1800) and mapping (0x1600/0x1A00) objects are writable by SDO
# with the CiA 301 dynamic mapping sequence. Mappable objects (index, sub, size in bytes): the
# example OD has no application objects; this is all there is.
# The error register 0x1001 is TPDO-mappable only (example OD attribute ODA_TPDO since v4.1).
RPDO_MAPPABLE = ((0x1280, 1, 4), (0x1280, 2, 4))
TPDO_MAPPABLE = ((0x1001, 0, 1), (0x1200, 1, 4), (0x1200, 2, 4), (0x1280, 1, 4), (0x1280, 2, 4))
# 0x1280:01 and :02 (SDO client COB-IDs, 32 bit, read/write) are the loopback pair: they accept
# a value only with bits 11..29 zero (else SDO abort 0x06090030; an RPDO carrying such a value is
# simply not applied). Keep bit 31 set so the SDO client stays disabled.
LOOPBACK_OBJECT = (0x1280, 1)
LOOPBACK_SECOND = (0x1280, 2)
LOOPBACK_INVALID_MASK = 0x3FFFF800
# A valid COB-ID (bit 31 clear) is refused with 0x06090030 when its CAN-ID is restricted by CiA 301.
# CANopenNode 76b43c8 accepted LOOPBACK_RESTRICTED_VALID; v4.1 refuses it (CO_IS_RESTRICTED_CAN_ID).
# An RPDO mapping 16 bits of 0x1280:01 zero-fills bit 31, so it can only carry a free CAN-ID.
LOOPBACK_RESTRICTED_VALID = 0x00000123
LOOPBACK_FREE_VALID = 0x00000234
LOOPBACK_DISABLED = 0x80000000

# --- PDO-length EMCY (test_canopend_pdo_length_emcy) -------------------------------------------
# RPDO shorter than its mapping: EMCY 0x8210; longer: 0x8220 (the plan expected 0x8210 only for
# the short case). EMCY frame on 0x082: code (LE 2 bytes), error register, then the manufacturer
# field = error bit 0x04 (RPDO wrong length) and info code = mapped length in bytes (LE 4).
# Error register stays 0 and the NMT state is not changed (heartbeat stays Operational).
EMCY_PDO_LENGTH_SHORT = 0x8210
EMCY_PDO_LENGTH_LONG = 0x8220
EMCY_RESET_CODE = 0x0000  # sent on the next RPDO of the right length after a short one
EMCY_ERROR_BIT_RPDO_LENGTH = 0x04
EMCY_ERROR_REGISTER = 0x00
# The storage-failure EMCY of a first start without storage files (gate check 3).
EMCY_NON_VOLATILE_MEMORY = 0x5000

# --- Reset (test_canopend_reset_node_behaviour, ..._reset_communication_...) -------------------
# Reset Node ends the process by itself with exit status 0 (CO_RESET_APP; `-r` would reboot
# Linux and is never used). After the NMT frame it puts nothing on the bus (no boot-up, no
# heartbeat); its log ends with LOG_FINISHED. The PDO configuration is not stored, so a restart
# with the same storage shows 0x1017 = SEEDED_HEARTBEAT_MS and the default PDO configuration.
RESET_NODE_EXIT_CODE = 0
# Reset Communication keeps the process: boot-up 0x00 on 0x702 is followed by 0x7F.

# --- SYNC and RPDO (test_canopend_sync_rpdo_applied_at_sync) -----------------------------------
# A type 0-240 RPDO is applied at a SYNC, not on reception; a type 255 RPDO on reception. A TPDO
# of type 1 sent at that same SYNC already carries the applied value (RPDO is processed first);
# pinned: the TPDO bytes after each SYNC equal the sequence below.
# From a fresh start, RPDO value v then SYNC, repeated for v = 1..8, leaves 1, 2, ..., 8.
# Every SYNC applies the RPDO received before it.
# CANopenNode 76b43c8 (CANopenLinux v4.0) read every second one from the wrong RX buffer and
# left 1, 1, 3, 3, 5, 5, 7, 7 (301/CO_PDO.c:811, fixed by PR #320, 84487ef, in v4.1).
SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC = (1, 2, 3, 4, 5, 6, 7, 8)
