# pylint: disable=too-many-instance-attributes
"""Register definitions for the TMC2300 driver.

The TMC2300 largely mirrors the TMC220x register map but trims GCONF/IOIN/
DRV_STATUS and adjusts COOLCONF. Only the fields present in the datasheet
excerpt provided are defined; unspecified bits remain reserved.
"""

from . import _tmc_shared_regs as shared
from ._tmc_reg import TmcRegField
from ._tmc220x_reg import (
    GStat,
    IfCnt,
    IHoldIRun,
    TPowerDown,
    TStep,
    TPwmThrs,
    VActual,
    MsCnt,
    ChopConf,
    PwmConf,
)
from ._tmc2209_reg import TCoolThrs, SgThrs, SgResult


class GConf(shared.GConf):
    """GCONF register class (TMC2300 specific)"""

    ADDR = 0x0

    extcap: bool
    shaft: bool
    diag_index: bool
    diag_step: bool
    multistep_filt: bool
    test_mode: bool
    _REG_MAP = (
        # Bits 0 and 2 are documented as "set to 0"; keep them reserved.
        TmcRegField("extcap", 1, 0x1, bool, None, ""),
        TmcRegField("shaft", 3, 0x1, bool, None, ""),
        TmcRegField("diag_index", 4, 0x1, bool, None, ""),
        TmcRegField("diag_step", 5, 0x1, bool, None, ""),
        TmcRegField("multistep_filt", 6, 0x1, bool, None, ""),
        TmcRegField("test_mode", 7, 0x1, bool, None, ""),
    )


class Ioin(shared.Ioin):
    """IOIN (INPUT) register class (TMC2300 specific)"""

    ADDR = 0x6
    DRIVER_NAME = "TMC2300"

    version: int
    en: bool
    nstdby: bool
    ad0: bool
    ad1: bool
    diag: bool
    stepper: bool
    pdn_uart: bool
    mode: bool
    step: bool
    dir: bool
    comp_a1a2: bool
    comp_b1b2: bool
    _REG_MAP = (
        TmcRegField("en", 0, 0x1, bool, None, ""),
        TmcRegField("nstdby", 1, 0x1, bool, None, ""),
        TmcRegField("ad0", 2, 0x1, bool, None, ""),
        TmcRegField("ad1", 3, 0x1, bool, None, ""),
        TmcRegField("diag", 4, 0x1, bool, None, ""),
        TmcRegField("stepper", 5, 0x1, bool, None, ""),
        TmcRegField("pdn_uart", 6, 0x1, bool, None, ""),
        TmcRegField("mode", 7, 0x1, bool, None, ""),
        TmcRegField("step", 8, 0x1, bool, None, ""),
        TmcRegField("dir", 9, 0x1, bool, None, ""),
        TmcRegField("comp_a1a2", 10, 0x1, bool, None, ""),
        TmcRegField("comp_b1b2", 11, 0x1, bool, None, ""),
        TmcRegField("version", 24, 0xFF, int, None, ""),
    )


class DrvStatus(shared.DrvStatus):
    """DRV_STATUS register class (TMC2300 specific)"""

    ADDR = 0x6F

    stst: bool
    cs_actual: int
    t150: bool
    t120: bool
    olb: bool
    ola: bool
    s2vsb: bool
    s2vsa: bool
    s2gb: bool
    s2ga: bool
    ot: bool
    otpw: bool
    _REG_MAP = (
        TmcRegField("stst", 31, 0x1, bool, None, ""),
        TmcRegField("cs_actual", 16, 0x1F, int, None, ""),
        TmcRegField("t150", 9, 0x1, bool, None, ""),
        TmcRegField("t120", 8, 0x1, bool, None, ""),
        TmcRegField("olb", 7, 0x1, bool, None, ""),
        TmcRegField("ola", 6, 0x1, bool, None, ""),
        TmcRegField("s2vsb", 5, 0x1, bool, None, ""),
        TmcRegField("s2vsa", 4, 0x1, bool, None, ""),
        TmcRegField("s2gb", 3, 0x1, bool, None, ""),
        TmcRegField("s2ga", 2, 0x1, bool, None, ""),
        TmcRegField("ot", 1, 0x1, bool, None, ""),
        TmcRegField("otpw", 0, 0x1, bool, None, ""),
    )


class CoolConf(shared.CoolConf):
    """COOLCONF register class (TMC2300 specific)"""

    ADDR = 0x42

    seimin: bool
    sedn: int
    semax: int
    seup: int
    semin: int
    _REG_MAP = (
        TmcRegField("seimin", 15, 0x1, bool, None, ""),
        TmcRegField("sedn", 13, 0x3, int, None, ""),
        TmcRegField("semax", 8, 0xF, int, None, ""),
        TmcRegField("seup", 5, 0x3, int, None, ""),
        TmcRegField("semin", 0, 0xF, int, None, ""),
    )
