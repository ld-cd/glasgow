# Ref: Configuring Trion FPGAs
# Document Number: AN006
# Accession: G00133
#
# Ref: Configuring Titanium FPGAs
# Document Number: AN033
# Accession: G00134

from collections.abc import Buffer
import argparse
import asyncio

from amaranth import *
from amaranth.lib import enum, wiring, stream, data
from amaranth.utils import exact_log2

from glasgow.gateware.iostream import SimulatableDDRBuffer
from glasgow.gateware.cobs import encode, Decoder
from glasgow.gateware.ports import PortGroup
from glasgow.support import logging, bits
from glasgow.abstract import AbstractAssembly, GlasgowPin, ClockDivisor
from glasgow.applet.control.gpio import GPIOInterface
from glasgow.applet import GlasgowAppletError, GlasgowAppletV2


__all__ = ["EfinixSRAMInterface"]


class Command(data.Struct):
    class Opcode(enum.Enum, shape=4):
        DATA          = 0
        OUTPUT_ENABLE = 1
        OUTPUT_SET    = 2
        DELAY         = 3

    opcode: Opcode
    params: data.UnionLayout({
        "output_enable": data.StructLayout({
            "reset": 1,
            "cs":    1,
            "cck":   1,
            "cdi":   1,
        }),
        "output_set": data.StructLayout({
            "reset": 1,
            "cs":    1,
        })
    })


class Enframer(wiring.Component):
    def __init__(self, ports):
        self._ports = ports
        self._width = len(ports.cdi)

        super().__init__({
            "bitstream": wiring.In(stream.Signature(8)),
            "reset": wiring.In(1),
            "cs": wiring.In(1),
            "oe": wiring.In(data.StructLayout({
                "reset": 1,
                "cs":    1,
                "cck":   1,
                "cdi":   1,
            })),
            "divisor": wiring.In(16),
        })

    def elaborate(self, platform):
        m = Module()

        m.submodules.reset = reset = SimulatableDDRBuffer("o", self._ports.reset)
        m.submodules.cs    = cs    = SimulatableDDRBuffer("o", self._ports.cs)
        m.submodules.cck   = cck   = SimulatableDDRBuffer("o", self._ports.cck)
        m.submodules.cdi   = cdi   = SimulatableDDRBuffer("o", self._ports.cdi)

        m.d.sync += [
            reset.oe.eq(self.oe.reset),
            cs   .oe.eq(self.oe.cs),
            cck  .oe.eq(self.oe.cck),
            cdi  .oe.eq(self.oe.cdi),
        ]

        m.d.sync += [
            reset.o.eq(self.reset.replicate(2)),
            cs   .o.eq(self.cs   .replicate(2)),
        ]

        width = len(self._ports.cdi)
        cycle = Signal(range(8 // width), init=0)
        timer = Signal.like(self.divisor)
        data  = self.bitstream.payload[::-1]
        with m.If(self.bitstream.valid):
            m.d.sync += [
                timer   .eq(timer + 1),
                cck.o[0].eq(timer * 2 >  self.divisor),
                cck.o[1].eq(timer * 2 >= self.divisor),
                cdi.o[0].eq(data.word_select(cycle, width)[::-1]),
                cdi.o[1].eq(data.word_select(cycle, width)[::-1]),
            ]
            with m.If(timer == self.divisor):
                m.d.sync += [
                    cycle.eq(cycle + 1),
                    timer.eq(0),
                ]
                with m.If(cycle == (8 // width) - 1):
                    m.d.sync += cycle.eq(0)
                    m.d.comb += self.bitstream.ready.eq(1)

        return m


class EfinixConfigComponent(wiring.Component):
    def __init__(self, ports):
        self._ports = ports

        super().__init__({
            "control": wiring.In(stream.Signature(8)),
            "divisor": wiring.In(16),
        })

    def elaborate(self, platform):
        m = Module()

        m.submodules.enf = enf = Enframer(self._ports)
        m.d.comb += enf.divisor.eq(self.divisor)

        m.submodules.dec = dec = Decoder()
        wiring.connect(m, dec.i, wiring.flipped(self.control))

        command = Signal(Command)
        m.d.comb += command.eq(dec.o.p.data)

        timer = Signal.like(self.divisor)
        delay = Signal(8)

        with m.FSM():
            with m.State("COMMAND"):
                m.d.comb += dec.o.ready.eq(1)
                with m.If(dec.o.valid & dec.o.ready & ~dec.o.p.end):
                    with m.Switch(command.opcode):
                        with m.Case(Command.Opcode.DATA):
                            m.next = "DATA"
                        with m.Case(Command.Opcode.OUTPUT_ENABLE):
                            m.d.sync += enf.oe   .eq(command.params.output_enable)
                        with m.Case(Command.Opcode.OUTPUT_SET):
                            m.d.sync += enf.reset.eq(command.params.output_set.reset)
                            m.d.sync += enf.cs   .eq(command.params.output_set.cs)
                        with m.Case(Command.Opcode.DELAY):
                            m.next = "DELAY"

            with m.State("DATA"):
                with m.If(dec.o.valid & dec.o.p.end):
                    m.d.comb += dec.o.ready.eq(1)
                    m.next = "COMMAND"
                with m.Else():
                    m.d.comb += [
                        enf.bitstream.valid.eq(dec.o.valid & ~dec.o.p.end),
                        dec.o.ready.eq(enf.bitstream.ready),
                        enf.bitstream.p.eq(dec.o.p.data),
                    ]

            with m.State("DELAY"):
                with m.If(dec.o.valid & dec.o.p.end):
                    m.d.comb += dec.o.ready.eq(1)
                    m.next = "COMMAND"
                with m.Else():
                    m.d.sync += timer.eq(timer + 1)
                    with m.If(timer == self.divisor):
                        m.d.sync += timer.eq(0)
                        m.d.sync += delay.eq(delay + 1)
                        with m.If(delay + 1 == dec.o.p.data):
                            m.d.sync += delay.eq(0)
                            m.d.comb += dec.o.ready.eq(1)
                            m.next = "COMMAND"

        return m


class EfinixSRAMError(GlasgowAppletError):
    pass


class EfinixSRAMInterface:
    def __init__(self, logger: logging.Logger, assembly: AbstractAssembly, *,
                 creset: GlasgowPin, cs: GlasgowPin, cck: GlasgowPin, cdi: GlasgowPin,
                 cbus: GlasgowPin | None = None, cdone: GlasgowPin | None = None,
                 freset: GlasgowPin | None = None):
        self._logger = logger
        self._level  = logging.DEBUG if self._logger.name == __name__ else logging.TRACE

        self._width = len(cdi)
        component = assembly.add_submodule(EfinixConfigComponent(PortGroup(
            reset = assembly.add_port(~creset, name="reset"),
            cs    = assembly.add_port(~cs,     name="cs"),
            cck   = assembly.add_port( cck,    name="cck"),
            cdi   = assembly.add_port( cdi,    name="cdi"),
        )))

        self._control_pipe = assembly.add_out_pipe(component.control)
        self._config_clock = assembly.add_clock_divisor(
            component.divisor,
            ref_period=assembly.sys_clk_period,
            name="config_clock"
        )

        self._cbus_iface   = None
        self._cdone_iface  = None
        self._freset_iface = None
        if cbus is not None:
            self._cbus_iface   = GPIOInterface(logger, assembly, pins=cbus, name="cbus")
        if cdone is not None:
            self._cdone_iface  = GPIOInterface(logger, assembly, pins=(cdone,), name="cdone")
        if freset is not None:
            self._freset_iface = GPIOInterface(logger, assembly, pins=(~freset,), name="freset")

    def _log(self, message: str, *args):
        self._logger.log(self._level, "efinix: " + message, *args)

    @property
    def config_clock(self) -> ClockDivisor:
        return self._config_clock

    @property
    def width(self) -> int:
        return self._width

    async def send_packet(self, data: Buffer):
        await self._control_pipe.send(encode(data) + b"\x00")
        await self._control_pipe.flush()

    async def send_output_enable(self, *, creset: bool, cs: bool, cck: bool, cdi: bool):
        cmd = Command.const({
            "opcode": Command.Opcode.OUTPUT_ENABLE,
            "params": {
                "output_enable": {
                    "reset": creset, "cs": cs, "cck": cck, "cdi": cdi
                }
            }
        }).as_bits().to_bytes(1)
        await self.send_packet(cmd)

    async def send_output_set(self, *, creset: bool, cs: bool):
        cmd = Command.const({
            "opcode": Command.Opcode.OUTPUT_SET,
            "params": {
                "output_set": {
                    "reset": creset, "cs": cs,
                }
            }
        }).as_bits().to_bytes(1)
        await self.send_packet(cmd)

    async def send_data(self, data: Buffer):
        cmd = Command.const({
            "opcode": Command.Opcode.DATA,
        }).as_bits().to_bytes(1)
        await self.send_packet(cmd + data)

    async def send_delay(self, delay: int):
        cmd = Command.const({
            "opcode": Command.Opcode.DELAY,
        }).as_bits().to_bytes(1)

        head = b"\xff" * (delay // 255)
        tail = (delay % 255).to_bytes(1)

        await self.send_packet(cmd + head + tail)

    async def load(self, bitstream: Buffer):
        """Load :py:`bitstream` into configuration SRAM.

        Raises
        ------
        EfinixError
            If the CDONE pin is present and was not asserted within 10 ms after the bitstream
            has been shifted in.
        """
        await self.send_output_enable(creset=True, cs=False, cdi=False, cck=False)
        await self.send_output_set(creset=True, cs=False)

        # Wait TRESET_MIN before enabling other IO Buffers, realistically flushing pipes
        # will take this long but in theory we can clock the interface up to 100MHz
        await self.send_delay(1 + int(320e-9 / (await self.config_clock.get_frequency())))
        await self.send_output_enable(creset=True, cs=True, cdi=True, cck=True)
        await self.send_output_set(creset=True, cs=True)

        if self._freset_iface is not None:
            await self._freset_iface.output(0, True)
        if self._cbus_iface is not None:
            setting = ~bits.bits.from_int(exact_log2(self.width), length=3)
            await self._cbus_iface.output(0, setting[0])
            await self._cbus_iface.output(1, setting[1])
            await self._cbus_iface.output(2, setting[2])

        # Clock out a byte so CCK is idling high, again probably not strictly necessary
        # as the config engine waits for a sync sequence before doing anything
        await self.send_data(b"\xff")
        await self.send_output_set(creset=False, cs=True)

        # Wait at least TD_MIN before we start clocking data out
        await self.send_delay(1 + int(1.2e-6 / (await self.config_clock.get_frequency())))
        await self.send_data(bitstream)

        # Datasheeet specifies at least 120 CCK clocks after loading the last of the bitstream
        await self.send_data(b"\xff" * 128 * self.width)

        # Eveything but creset is a GPIO, so tristate before user mode gets entered
        await self.send_output_set(creset=False, cs=False)
        await self.send_output_enable(creset=False, cs=False, cdi=False, cck=False)

        if self._cbus_iface is not None:
            await self._cbus_iface.input(0)
            await self._cbus_iface.input(1)
            await self._cbus_iface.input(2)

        if self._freset_iface is not None:
            await self._freset_iface.output(0, False)
            await self._freset_iface.input(0)

        if self._cdone_iface is not None:
            self._log("waiting for CDONE")
            for _ in range(10):
                if await self._cdone_iface.get(0):
                    return
                await asyncio.sleep(0.001)
            raise EfinixSRAMError("FPGA failed to configure")
        else:
            self._log("waiting for CDONE (absent)")


class ProgramEfinixSRAMApplet(GlasgowAppletV2):
    logger = logging.getLogger(__name__)
    help = "program SRAM of Efinix FPGAs"
    description = """
    Program the volatile bitstream memory of Efinix FPGAs.
    """
    required_revision = "C0"

    @classmethod
    def add_build_arguments(cls, parser, access):
        access.add_voltage_argument(parser)
        access.add_pins_argument(parser, "creset", default=True, required=True)
        access.add_pins_argument(parser, "cs",     default=True, required=True,
                                 help="Called SS_N on some boards")
        access.add_pins_argument(parser, "cck",    default=True, required=True)
        access.add_pins_argument(parser, "cdi",    default=True, required=True, width=range(1, 9))
        access.add_pins_argument(parser, "cbus",   width=3)
        access.add_pins_argument(parser, "cdone")

        access.add_pins_argument(parser, "freset",
                                 help="FTDI Reset present on all the efinix dev boards")

    def build(self, args):
        # TODO: Remove once versions of yosys with #4349 are dropped ----
        #
        # Credit: zyp (https://paste.jvnv.net/view/QfIuO)
        #
        # Workaround for https://github.com/YosysHQ/yosys/issues/4349.
        #
        # Signal renaming in Yosys sometimes causes name conflicts when both signals foo and foo[0]
        # exist in the original design, which happens when we use data.ArrayLayout. We can work
        # around this by monkeypatching Format.Array to generate struct suffixes for array fields,
        # i.e. foo.0 instead of foo[0].
        from amaranth import Format
        def Array(value, /, fields):
            return Format.Struct(value, {str(i): f for i, f in enumerate(fields)})
        Format.Array = Array
        # ---------------------------------------------------------------
        if len(args.cdi) not in [1, 2, 4, 8]:
            raise EfinixSRAMError("CDI must be 1, 2, 4, or 8 pins wide")
        with self.assembly.add_applet(self):
            self.assembly.use_voltage(args.voltage)
            self.efinix_iface = EfinixSRAMInterface(
                self.logger, self.assembly,
                creset=args.creset, cs=args.cs, cck=args.cck, cdi=args.cdi,
                cbus=args.cbus, cdone=args.cdone
            )

    @classmethod
    def add_setup_arguments(cls, parser):
        # T55+/Nook has an errata that limits the config clock to 10MHz for the lower
        # speed grades and 12.5MHz for the higher speed grades as opposed to it being
        # 25MHz in X1 mode for every other part in every speed grade. In practice
        # this appears to work out to 48MHz for T20/Rebecca
        parser.add_argument(
            "-f", "--frequency", metavar="FREQ", type=int, default=9_600,
            help="set SCK frequency to FREQ kHz (default: %(default)s)")

    async def setup(self, args):
        await self.efinix_iface.config_clock.set_frequency(args.frequency * 1000)

    @classmethod
    def add_run_arguments(cls, parser):
        group = parser.add_mutually_exclusive_group(required=True)
        group.add_argument(
            "--hex", metavar="HEX", type=argparse.FileType("r"),
            help=".hex file emitted by the toolchain")
        group.add_argument(
            "--bin", metavar="BINARY", type=argparse.FileType("rb"),
            help="a binary serialization of the .hex file, emitted as .bin by the toolchain")

    async def run(self, args):
        self.logger.info("loading bitstream")
        if args.bin is not None:
            await self.efinix_iface.load(args.bin.read())
        else:
            # The fact that this is the bitstream interchange format is frankly
            # baffling to me
            parsed = b"".join([
                int(l.strip(), base=16).to_bytes(1)
                for l in args.hex.readlines()
            ])
            await self.efinix_iface.load(parsed)

    @classmethod
    def tests(cls):
        from . import test
        return test.ProgramEfinixSRAMAppletAppletTestCase
