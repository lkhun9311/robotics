# SPDX-License-Identifier: Apache-2.0
"""CiA 301 tables for the interop verdicts, written from the standard's text (B16).

This module is the expectation side of the interop matrix. It does not import our C++ code, the
EDS tools or wire.py, so a misreading shared by the drive and wire.py cannot make both sides
agree. All multi-byte fields are little-endian.
"""

# predefined connection set: function code bases; COB-ID = base + node-ID.
NMT_COB = 0x000
SYNC_COB = 0x080
EMCY_BASE = 0x080
TPDO_BASE = {1: 0x180, 2: 0x280, 3: 0x380, 4: 0x480}
RPDO_BASE = {1: 0x200, 2: 0x300, 3: 0x400, 4: 0x500}
SDO_TX_BASE = 0x580  # server -> client
SDO_RX_BASE = 0x600  # client -> server
HEARTBEAT_BASE = 0x700

# NMT module control: command specifiers (byte 0; byte 1 is the node-ID, 0 = all).
NMT_START = 0x01
NMT_STOP = 0x02
NMT_ENTER_PRE_OPERATIONAL = 0x80
NMT_RESET_NODE = 0x81
NMT_RESET_COMMUNICATION = 0x82

# heartbeat / boot-up protocol: the state byte (bit 7 reserved, 0).
STATE_BOOT_UP = 0x00
STATE_STOPPED = 0x04
STATE_OPERATIONAL = 0x05
STATE_PRE_OPERATIONAL = 0x7F

# NMT state machine: the state a node is in after each command.
# After a reset the node runs Initialisation, sends the boot-up message and enters Pre-operational
# by itself.
STATE_AFTER = {
    NMT_START: STATE_OPERATIONAL,
    NMT_STOP: STATE_STOPPED,
    NMT_ENTER_PRE_OPERATIONAL: STATE_PRE_OPERATIONAL,
    NMT_RESET_NODE: STATE_PRE_OPERATIONAL,
    NMT_RESET_COMMUNICATION: STATE_PRE_OPERATIONAL,
}
RESETS = (NMT_RESET_NODE, NMT_RESET_COMMUNICATION)

# services per NMT state: SDO and EMCY in Pre-operational and Operational;
# PDO in Operational only; in Stopped only NMT and error control (heartbeat).
SDO_STATES = (STATE_PRE_OPERATIONAL, STATE_OPERATIONAL)
PDO_STATES = (STATE_OPERATIONAL,)
EMCY_STATES = (STATE_PRE_OPERATIONAL, STATE_OPERATIONAL)

# SDO protocols: the command byte. ccs/scs in bits 7..5; for an expedited initiate,
# n (bits 3..2) = 4 - data bytes, e (bit 1) = expedited, s (bit 0) = size indicated.
CCS_DOWNLOAD_INITIATE = 1
CCS_UPLOAD_INITIATE = 2
SCS_UPLOAD_INITIATE = 2
SCS_DOWNLOAD_INITIATE = 3
CS_ABORT = 4

# SDO abort codes (those the matrix uses, plus the client timeout).
ABORT_TIMEOUT = 0x05040000  # SDO protocol timed out
ABORT_READ_ONLY = 0x06010002  # Attempt to write a read only object
ABORT_NO_OBJECT = 0x06020000  # Object does not exist in the object dictionary
ABORT_LENGTH = 0x06070010  # Data type does not match, length of service parameter does not match
ABORT_VALUE_TOO_HIGH = 0x06090031  # Value of parameter written too high (download only)
ABORT_INVALID_VALUE = 0x06090030  # Invalid value for parameter (download only)

# EMCY write: 8 bytes = emergency error code (UNSIGNED16), error register (1001h,
# UNSIGNED8), 5 bytes manufacturer-specific.
# Code 0000h = error reset or no error.
# Error register (1001h): bit 0 = generic error, set whenever any error is present.
EMCY_ERROR_RESET = 0x0000
ERROR_REGISTER_NONE = 0x00
ERROR_REGISTER_GENERIC = 0x01


def _le(value, size):
    return list((value & ((1 << (8 * size)) - 1)).to_bytes(size, "little"))


def _multiplexer(index, sub):
    return _le(index, 2) + [sub]


def nmt(cs, node):
    return NMT_COB, bytes([cs, node])


def heartbeat(node, state):
    return HEARTBEAT_BASE + node, bytes([state])


def sdo_upload_request(node, index, sub):
    return SDO_RX_BASE + node, bytes([CCS_UPLOAD_INITIATE << 5] + _multiplexer(index, sub) + [0] * 4)


def sdo_upload_response(node, index, sub, size, value):
    """Expedited initiate upload response carrying `size` bytes (e = s = 1)."""
    cmd = (SCS_UPLOAD_INITIATE << 5) | ((4 - size) << 2) | 0b11
    return SDO_TX_BASE + node, bytes([cmd] + _multiplexer(index, sub) +
                                     (_le(value, size) + [0] * 4)[:4])


def sdo_download_request(node, index, sub, size, value):
    """Expedited initiate download request with `size` data bytes (e = s = 1)."""
    cmd = (CCS_DOWNLOAD_INITIATE << 5) | ((4 - size) << 2) | 0b11
    return SDO_RX_BASE + node, bytes([cmd] + _multiplexer(index, sub) +
                                     (_le(value, size) + [0] * 4)[:4])


def sdo_download_response(node, index, sub):
    return SDO_TX_BASE + node, bytes([SCS_DOWNLOAD_INITIATE << 5] + _multiplexer(index, sub) +
                                     [0] * 4)


def sdo_abort(node, index, sub, code):
    """Abort transfer sent by the server (on the server-to-client identifier)."""
    return SDO_TX_BASE + node, bytes([CS_ABORT << 5] + _multiplexer(index, sub) + _le(code, 4))


def is_download_initiate(data):
    return len(data) == 8 and data[0] >> 5 == CCS_DOWNLOAD_INITIATE


def multiplexer(data):
    """(index, sub) of an SDO frame."""
    return data[1] | data[2] << 8, data[3]


def emcy(node, code, register):
    return EMCY_BASE + node, bytes(_le(code, 2) + [register] + [0] * 5)
