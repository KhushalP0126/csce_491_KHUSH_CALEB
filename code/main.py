# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Part of the University of South Carolina CSCE491 course materials. Used by
# instructors for test case generators. Do not redistribute.

import sys
import os

###############################################################################

# This block is setup code that loads the utility library for this assignment.
# You shouldn't mess with it unless you know what you are doing.

# this is the directory where our code is (main.py)
code_dir = os.path.split(os.path.abspath(sys.argv[0]))[0]

# this will be ./.. - the project directory
parent_dir = os.path.split(code_dir)[0]

# the python utils live in ../utils/python_utils
python_utils_dir = os.path.join(parent_dir, "utils", "python_utils")

# append this to Python's import path and import it
sys.path.append(python_utils_dir)
from waves import Waves

###############################################################################

EXPECT_CMD = 0
EXPECT_LEN = 1
EXPECT_DATA = 2


class TransactionState:
    def __init__(self):
        self.state = EXPECT_CMD
        self.is_write = False
        self.is_stream = False
        self.address = 0
        self.remaining = 0
        self.data = []


def hex2(v):
    return "{:02x}".format(v & 0xFF)


def emit_transaction(tx):
    op = "WR" if tx.is_write else "RD"
    addr = hex2(tx.address)
    if not tx.is_stream:
        if not tx.data:
            return
        val = hex2(tx.data[0])
        sys.stdout.write("{} {} {}\n".format(op, addr, val))
        return

    sys.stdout.write("{} STREAM {}".format(op, addr))
    for val in tx.data:
        sys.stdout.write(" {}".format(hex2(val)))
    sys.stdout.write("\n")


def reset_transaction(tx):
    tx.state = EXPECT_CMD
    tx.is_write = False
    tx.is_stream = False
    tx.address = 0
    tx.remaining = 0
    tx.data = []


def handle_exchange(tx, mosi, miso):
    if tx.state == EXPECT_CMD:
        tx.address = (mosi >> 2) & 0x3F
        tx.is_write = ((mosi >> 1) & 0x1) != 0
        tx.is_stream = (mosi & 0x1) != 0
        tx.data = []
        if tx.is_stream:
            tx.state = EXPECT_LEN
        else:
            tx.remaining = 1
            tx.state = EXPECT_DATA
    elif tx.state == EXPECT_LEN:
        tx.remaining = mosi
        tx.data = []
        if tx.remaining == 0:
            emit_transaction(tx)
            reset_transaction(tx)
        else:
            tx.state = EXPECT_DATA
    elif tx.state == EXPECT_DATA:
        val = mosi if tx.is_write else miso
        tx.data.append(val)
        if tx.remaining > 0:
            tx.remaining -= 1
        if tx.remaining == 0:
            emit_transaction(tx)
            reset_transaction(tx)


def signal_at_idx(waves, signal, idx):
    return waves.data[idx][1][signal]


def main():
    w = Waves()
    w.loadText(sys.stdin.read())

    required = ["cpol", "cpha", "sclk", "ss", "mosi", "miso"]
    for name in required:
        if name not in w.sizes:
            sys.stderr.write("missing required signals\n")
            sys.exit(1)

    if w.samples() == 0:
        return

    cpol = signal_at_idx(w, "cpol", 0) & 0x1
    cpha = signal_at_idx(w, "cpha", 0) & 0x1
    sample_posedge = (cpol == 0 and cpha == 0) or (cpol == 1 and cpha == 1)

    prev_sclk = signal_at_idx(w, "sclk", 0) & 0x1
    prev_ss = signal_at_idx(w, "ss", 0) & 0x1
    ss_active = prev_ss == 0

    cur_mosi = 0
    cur_miso = 0
    bit_count = 0
    tx = TransactionState()

    for i in range(1, w.samples()):
        cur_sclk = signal_at_idx(w, "sclk", i) & 0x1
        cur_ss = signal_at_idx(w, "ss", i) & 0x1

        if cur_ss != prev_ss:
            ss_active = cur_ss == 0
            cur_mosi = 0
            cur_miso = 0
            bit_count = 0
            reset_transaction(tx)

        if ss_active and cur_sclk != prev_sclk:
            posedge = prev_sclk == 0 and cur_sclk == 1
            negedge = prev_sclk == 1 and cur_sclk == 0
            sample_edge = (sample_posedge and posedge) or (not sample_posedge and negedge)
            if sample_edge:
                mosi = signal_at_idx(w, "mosi", i) & 0x1
                miso = signal_at_idx(w, "miso", i) & 0x1
                cur_mosi = ((cur_mosi << 1) | mosi) & 0xFF
                cur_miso = ((cur_miso << 1) | miso) & 0xFF
                bit_count += 1
                if bit_count == 8:
                    handle_exchange(tx, cur_mosi, cur_miso)
                    cur_mosi = 0
                    cur_miso = 0
                    bit_count = 0

        prev_sclk = cur_sclk
        prev_ss = cur_ss


if __name__ == "__main__":
    main()
