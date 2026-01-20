# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
# pylint: disable=too-many-arguments
# pylint: disable=too-many-positional-arguments
"""Tmc2300 stepper driver module"""

from .tmc_220x import Tmc220x
from ._tmc_xxxx import TmcXXXX
from .tmc_logger import Loglevel
from ._tmc_exceptions import TmcDriverException
from .reg._tmc2300_reg import *
from .com._tmc_com_uart_base import TmcComUartBase
from .enable_control._tmc_ec_toff import TmcEnableControlToff
from .enable_control._tmc_ec_pin import TmcEnableControlPin
from .motion_control._tmc_mc_step_dir import TmcMotionControlStepDir
from .motion_control._tmc_mc_step_reg import TmcMotionControlStepReg
from .motion_control._tmc_mc_step_pwm_dir import TmcMotionControlStepPwmDir
from .motion_control._tmc_mc_vactual import TmcMotionControlVActual


class Tmc2300(Tmc220x):
    """Tmc2300 driver (UART)"""

    DRIVER_FAMILY = "TMC2300"
    SUPPORTED_COM_TYPES = (TmcComUartBase,)
    SUPPORTED_EC_TYPES = (TmcEnableControlToff, TmcEnableControlPin)
    SUPPORTED_MC_TYPES = (
        TmcMotionControlStepDir,
        TmcMotionControlStepReg,
        TmcMotionControlStepPwmDir,
        TmcMotionControlVActual,
    )

    def __init__(
        self,
        tmc_ec,
        tmc_mc,
        tmc_com=None,
        driver_address: int = 0,
        gpio_mode=None,
        loglevel: Loglevel = Loglevel.INFO,
        logprefix: str | None = None,
        log_handlers: list | None = None,
        log_formatter=None,
    ):
        if logprefix is None:
            logprefix = f"{self.DRIVER_FAMILY} {driver_address}"

        # Skip Tmc220x.__init__ to avoid instantiating the 220x register map; create
        # the base driver and attach the 2300-specific registers instead.
        TmcXXXX.__init__(
            self,
            tmc_ec,
            tmc_mc,
            tmc_com,
            driver_address,
            gpio_mode,
            loglevel,
            logprefix,
            log_handlers,
            log_formatter,
        )

        if self.tmc_com is not None:
            # Chip-specific registers
            self.gconf: GConf = GConf(self.tmc_com)
            self.gstat: GStat = GStat(self.tmc_com)
            self.ifcnt: IfCnt = IfCnt(self.tmc_com)
            self.ioin: Ioin = Ioin(self.tmc_com)
            self.ihold_irun: IHoldIRun = IHoldIRun(self.tmc_com)
            self.tpowerdown: TPowerDown = TPowerDown(self.tmc_com)
            self.tstep: TStep = TStep(self.tmc_com)
            self.tpwmthrs: TPwmThrs = TPwmThrs(self.tmc_com)
            self.vactual: VActual = VActual(self.tmc_com)
            self.mscnt: MsCnt = MsCnt(self.tmc_com)
            self.chopconf: ChopConf = ChopConf(self.tmc_com)
            self.pwmconf: PwmConf = PwmConf(self.tmc_com)
            self.drvstatus: DrvStatus = DrvStatus(self.tmc_com)
            # StallGuard / coolStep registers
            self.tcoolthrs: TCoolThrs = TCoolThrs(self.tmc_com)
            self.sgthrs: SgThrs = SgThrs(self.tmc_com)
            self.sgresult: SgResult = SgResult(self.tmc_com)
            self.coolconf: CoolConf = CoolConf(self.tmc_com)

        super()._init()

    # Feature differences -------------------------------------------------
    def get_spreadcycle(self) -> bool:  # type: ignore[override]
        """TMC2300 does not expose en_spreadcycle; always stealthChop-style."""
        return False

    def set_spreadcycle(self, en: bool):  # type: ignore[override]
        raise TmcDriverException("TMC2300 does not support spreadCycle selection via UART")

    def set_microstepping_resolution(self, mres: int):  # type: ignore[override]
        """TMC2300 lacks mstep_reg_select; set resolution and return."""
        super().set_microstepping_resolution(mres)

    def set_mstep_resolution_reg_select(self, en: bool):  # type: ignore[override]
        # Not available on TMC2300; ignore.
        return

    # Current configuration ----------------------------------------------
    def set_current_peak(
        self,
        run_current: int,
        hold_current_multiplier: float = 0.5,
        hold_current_delay: int = 10,
        pdn_disable: bool = True,
    ) -> int:  # type: ignore[override]
        """sets the Peak current for the motor.

        Uses the same calculation as 220x but skips i_scale_analog/internal_rsense,
        which are not present on TMC2300. VSENSE toggling is retained.
        """

        self.tmc_logger.log(f"Desired peak current: {run_current} mA", Loglevel.DEBUG)

        cs_irun = 0
        rsense = 0.11
        vfs = 0.325

        def calc_cs_irun(run_current_ma: int, rsense_ohm: float, vfs_volt: float) -> float:
            return 32.0 * run_current_ma / 1000.0 * (rsense_ohm + 0.02) / vfs_volt - 1

        def calc_run_current(cs_val: float, rsense_ohm: float, vfs_volt: float) -> float:
            return (cs_val + 1) / 32.0 * vfs_volt / (rsense_ohm + 0.02) * 1000

        cs_irun = calc_cs_irun(run_current, rsense, vfs)

        if cs_irun < 16:
            self.tmc_logger.log("CS too low; switching to VSense True", Loglevel.INFO)
            vfs = 0.180
            cs_irun = calc_cs_irun(run_current, rsense, vfs)
            self.set_vsense(True)
        else:
            self.tmc_logger.log("CS in range; using VSense False", Loglevel.INFO)
            self.set_vsense(False)

        cs_irun = min(max(cs_irun, 0), 31)
        cs_ihold = hold_current_multiplier * cs_irun

        cs_irun = round(cs_irun)
        cs_ihold = round(cs_ihold)
        hold_current_delay = round(hold_current_delay)

        self.tmc_logger.log(f"cs_irun: {cs_irun}", Loglevel.INFO)
        self.tmc_logger.log(f"CS_IHold: {cs_ihold}", Loglevel.INFO)
        self.tmc_logger.log(f"Delay: {hold_current_delay}", Loglevel.INFO)

        run_current_actual = calc_run_current(cs_irun, rsense, vfs)
        self.tmc_logger.log(
            f"Calculated theoretical peak current after gscaler: {run_current_actual} mA",
            Loglevel.DEBUG,
        )

        self._set_irun_ihold(cs_ihold, cs_irun, hold_current_delay)

        if pdn_disable and hasattr(self, "gconf"):
            # TMC2300 GCONF lacks pdn_disable; ignore silently.
            try:
                self.gconf.modify("pdn_disable", True)
            except Exception:  # pragma: no cover - optional bit
                pass

        return round(run_current_actual)

    # Capabilities not exposed on 2300 -----------------------------------
    def get_iscale_analog(self) -> bool:  # type: ignore[override]
        raise TmcDriverException("TMC2300 has no i_scale_analog bit")

    def set_iscale_analog(self, en: bool):  # type: ignore[override]
        raise TmcDriverException("TMC2300 has no i_scale_analog bit")

    def get_internal_rsense(self) -> bool:  # type: ignore[override]
        raise TmcDriverException("TMC2300 has no internal_rsense bit")

    def set_internal_rsense(self, en: bool):  # type: ignore[override]
        raise TmcDriverException("TMC2300 has no internal_rsense bit")
