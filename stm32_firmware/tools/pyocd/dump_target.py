#!/usr/bin/env python3
"""
Dump live STM32F407 target state through pyOCD.

Works with DAPLink / CMSIS-DAP and ST-Link probes.

Output is deliberately plain text so that an AI agent can read real hardware
values without a human describing the debugger window.

Usage (normally called through tools/pyocd/dump.ps1):
    python dump_target.py --target stm32f407vg
"""

from __future__ import annotations

import argparse
import sys

# name, base address, word count
BLOCKS = [
    ("SCB ICSR",                                 0xE000ED04, 1),
    ("SCB CFSR/HFSR/DFSR/MMFAR/BFAR",            0xE000ED28, 5),
    ("RCC CR/PLLCFGR/CFGR/CIR",                  0x40023800, 4),
    ("GPIOA MODER..ODR",                         0x40020000, 8),
    ("GPIOB MODER..ODR",                         0x40020400, 8),
    ("GPIOC MODER..ODR",                         0x40020800, 8),
    ("GPIOD MODER..ODR",                         0x40020C00, 8),
    ("GPIOE MODER..ODR",                         0x40021000, 8),
    ("TIM2 CR1..ARR",                            0x40000000, 12),
    ("TIM2 CCR1..CCR4",                          0x40000034, 4),
    ("CAN1 MCR/MSR/TSR/RF0R/IER/ESR/BTR/TIM",    0x40006400, 8),
    ("CAN1 TMI0.. (tx mailbox + filters)",       0x40006500, 8),
]

CFSR_BITS = {
    0: "IACCVIOL - instruction access violation",
    1: "DACCVIOL - data access violation",
    3: "MUNSTKERR - fault unstacking on exception return",
    4: "MSTKERR - fault stacking on exception entry",
    5: "MLSPERR - lazy FP state preservation fault",
    7: "MMARVALID - MMFAR holds a valid fault address",
    8: "IBUSERR - instruction bus error",
    9: "PRECISERR - precise data bus error",
    10: "IMPRECISERR - imprecise data bus error",
    11: "UNSTKERR - fault unstacking on exception return",
    12: "STKERR - fault stacking on exception entry",
    13: "LSPERR - lazy FP state preservation fault",
    15: "BFARVALID - BFAR holds a valid fault address",
    16: "UNDEFINSTR - undefined instruction",
    17: "INVSTATE - invalid EPSR / not in Thumb state",
    18: "INVPC - invalid PC load",
    19: "NOCP - coprocessor access (FPU not enabled?)",
    24: "UNALIGNED - unaligned memory access",
    25: "DIVBYZERO - divide by zero",
}

HFSR_BITS = {
    1: "VECTTBL - fault during vector table read",
    30: "FORCED - fault escalated to HardFault",
    31: "DEBUGEVT - debug event",
}

CORE_REGS = ["pc", "lr", "sp", "xpsr", "r0", "r1", "r2", "r3", "r12"]


def decode(value: int, table: dict) -> list:
    out = []
    for bit, text in sorted(table.items()):
        if value & (1 << bit):
            out.append(text)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Dump STM32 target state via pyOCD")
    ap.add_argument("-t", "--target", default="stm32f407vg",
                    help="pyOCD target name (default: stm32f407vg)")
    ap.add_argument("-u", "--uid", default=None, help="probe unique id, if several are attached")
    ap.add_argument("--no-resume", action="store_true",
                    help="leave the target halted after the dump")
    ap.add_argument("--no-halt", action="store_true",
                    help="do not halt the core (memory reads only, registers may be stale)")
    args = ap.parse_args()

    try:
        from pyocd.core.helpers import ConnectHelper
    except ImportError:
        print("ERROR: pyocd is not importable. Run this through tools/pyocd/dump.ps1")
        return 3

    print("== pyOCD target dump ==")
    print("target : %s" % args.target)

    # blocking=False is essential. With the pyOCD default (True) this call waits
    # forever for a probe to appear, which would hang an automated / agent-driven
    # invocation instead of returning a clean error.
    session = ConnectHelper.session_with_chosen_probe(
        blocking=False,
        target_override=args.target,
        unique_id=args.uid,
        options={"resume_on_disconnect": not args.no_resume},
    )

    if session is None:
        print("ERROR: no debug probe found, or connection failed.")
        print("       Check that a DAPLink or ST-Link is plugged in, then run: pyocd list")
        return 2

    try:
        with session:
            target = session.target
            print("probe  : %s" % getattr(session.probe, "product_name", "?"))

            if not args.no_halt:
                target.halt()
                print("state  : halted by this dump")

            print("")
            print("-- core registers --")
            for name in CORE_REGS:
                try:
                    val = target.read_core_register(name)
                    print("  %-5s = 0x%08X" % (name, val))
                except Exception as exc:  # noqa: BLE001
                    print("  %-5s = <unreadable: %s>" % (name, exc))

            print("")
            print("-- peripherals --")
            for name, base, count in BLOCKS:
                try:
                    words = target.read_memory_block32(base, count)
                except Exception as exc:  # noqa: BLE001
                    print("  %s @ 0x%08X : <read failed: %s>" % (name, base, exc))
                    continue
                print("  %s @ 0x%08X" % (name, base))
                for i in range(0, len(words), 4):
                    chunk = words[i:i + 4]
                    addr = base + i * 4
                    line = "    +0x%02X: " % (i * 4)
                    line += "  ".join("%08X" % w for w in chunk)
                    print(line)

            # ---- fault analysis: the whole point of this dump ----
            try:
                icsr = target.read_memory_block32(0xE000ED04, 1)[0]
                cfsr = target.read_memory_block32(0xE000ED28, 1)[0]
                hfsr = target.read_memory_block32(0xE000ED2C, 1)[0]
                mmfar = target.read_memory_block32(0xE000ED34, 1)[0]
                bfar = target.read_memory_block32(0xE000ED38, 1)[0]

                print("")
                print("-- fault analysis --")
                print("  ICSR   = 0x%08X   VECTACTIVE=%d (0 = thread mode)"
                      % (icsr, icsr & 0x1FF))
                print("  HFSR   = 0x%08X" % hfsr)
                print("  CFSR   = 0x%08X" % cfsr)

                hf = decode(hfsr, HFSR_BITS)
                cf = decode(cfsr, CFSR_BITS)

                if hfsr or cfsr:
                    print("  *** FAULT RECORDED ***")
                    for t in hf:
                        print("    HFSR: " + t)
                    for t in cf:
                        print("    CFSR: " + t)
                    if cfsr & (1 << 7):
                        print("    MMFAR: 0x%08X  <-- faulting data address" % mmfar)
                    if cfsr & (1 << 15):
                        print("    BFAR : 0x%08X  <-- faulting bus address" % bfar)
                else:
                    print("  no fault latched (CFSR/HFSR are clear)")

                print("")
                print("  TIP: on HardFault with no latched bits, read active task stack:")
                print("       FreeRTOS current TCB stack pointer -> unwind manually.")
            except Exception as exc:  # noqa: BLE001
                print("  fault analysis unavailable: %s" % exc)

            if not args.no_halt and not args.no_resume:
                target.resume()
                print("")
                print("target resumed")

        return 0

    except Exception as exc:  # noqa: BLE001
        print("ERROR: %s" % exc)
        return 4


if __name__ == "__main__":
    sys.exit(main())
