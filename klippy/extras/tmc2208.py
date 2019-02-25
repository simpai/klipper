# TMC2208 UART communication and configuration
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, collections
import tmc2130

TMC_FREQUENCY=12000000.

Registers = {
    "GCONF": 0x00, "GSTAT": 0x01, "IFCNT": 0x02, "SLAVECONF": 0x03,
    "OTP_PROG": 0x04, "OTP_READ": 0x05, "IOIN": 0x06, "FACTORY_CONF": 0x07,
    "IHOLD_IRUN": 0x10, "TPOWERDOWN": 0x11, "TSTEP": 0x12, "TPWMTHRS": 0x13,
    "VACTUAL": 0x22, "MSCNT": 0x6a, "MSCURACT": 0x6b, "CHOPCONF": 0x6c,
    "DRV_STATUS": 0x6f, "PWMCONF": 0x70, "PWM_SCALE": 0x71, "PWM_AUTO": 0x72
}

ReadRegisters = [
    "GCONF", "GSTAT", "IFCNT", "OTP_READ", "IOIN", "FACTORY_CONF", "TSTEP",
    "MSCNT", "MSCURACT", "CHOPCONF", "DRV_STATUS",
    "PWMCONF", "PWM_SCALE", "PWM_AUTO"
]

Fields = {}

Fields["GCONF"] = {
    "I_scale_analog":      0x01,
    "internal_Rsense":     0x01 << 1,
    "en_spreadCycle":      0x01 << 2,
    "shaft":               0x01 << 3,
    "index_otpw":          0x01 << 4,
    "index_step":          0x01 << 5,
    "pdn_disable":         0x01 << 6,
    "mstep_reg_select":    0x01 << 7,
    "multistep_filt":      0x01 << 8,
    "test_mode":           0x01 << 9
}
Fields["GSTAT"] = {
    "reset":               0x01,
    "drv_err":             0x01 << 1,
    "uv_cp":               0x01 << 2
}
Fields["IFCNT"] = {
    "IFCNT":               0xff
}
Fields["SLAVECONF"] = {
    "SENDDELAY":           0x0f << 8
}
Fields["OTP_PROG"] = {
    "OTPBIT":              0x07,
    "OTPBYTE":             0x03 << 4,
    "OTPMAGIC":            0xff << 8
}
Fields["OTP_READ"] = {
    "OTP_FCLKTRIM":        0x1f,
    "otp_OTTRIM":          0x01 << 5,
    "otp_internalRsense":  0x01 << 6,
    "otp_TBL":             0x01 << 7,
    "OTP_PWM_GRAD":        0x0f << 8,
    "otp_pwm_autograd":    0x01 << 12,
    "OTP_TPWMTHRS":        0x07 << 13,
    "otp_PWM_OFS":         0x01 << 16,
    "otp_PWM_REG":         0x01 << 17,
    "otp_PWM_FREQ":        0x01 << 18,
    "OTP_IHOLDDELAY":      0x03 << 19,
    "OTP_IHOLD":           0x03 << 21,
    "otp_en_spreadCycle":  0x01 << 23
}
# IOIN mapping depends on the driver type (SEL_A field)
# TMC222x (SEL_A == 0)
Fields["IOIN@TMC222x"] = {
    "PDN_UART":            0x01 << 1,
    "SPREAD":              0x01 << 2,
    "DIR":                 0x01 << 3,
    "ENN":                 0x01 << 4,
    "STEP":                0x01 << 5,
    "MS1":                 0x01 << 6,
    "MS2":                 0x01 << 7,
    "SEL_A":               0x01 << 8,
    "VERSION":             0xff << 24
}
# TMC220x (SEL_A == 1)
Fields["IOIN@TMC220x"] = {
    "ENN":                 0x01,
    "MS1":                 0x01 << 2,
    "MS2":                 0x01 << 3,
    "DIAG":                0x01 << 4,
    "PDN_UART":            0x01 << 6,
    "STEP":                0x01 << 7,
    "SEL_A":               0x01 << 8,
    "DIR":                 0x01 << 9,
    "VERSION":             0xff << 24,
}
Fields["FACTORY_CONF"] = {
    "FCLKTRIM":            0x1f,
    "OTTRIM":              0x03 << 8
}
Fields["IHOLD_IRUN"] = {
    "IHOLD":               0x1f,
    "IRUN":                0x1f << 8,
    "IHOLDDELAY":          0x0f << 16
}
Fields["TPOWERDOWN"] = {
    "TPOWERDOWN":          0xff
}
Fields["TSTEP"] = {
    "TSTEP":               0xfffff
}
Fields["TPWMTHRS"] = {
    "TPWMTHRS":            0xfffff
}
Fields["VACTUAL"] = {
    "VACTUAL":             0xffffff
}
Fields["MSCNT"] = {
    "MSCNT":               0x3ff
}
Fields["MSCURACT"] = {
    "CUR_A":               0x1ff,
    "CUR_B":               0x1ff << 16
}
Fields["CHOPCONF"] = {
    "toff":                0x0f,
    "hstrt":               0x07 << 4,
    "hend":                0x0f << 7,
    "TBL":                 0x03 << 15,
    "vsense":              0x01 << 17,
    "MRES":                0x0f << 24,
    "intpol":              0x01 << 28,
    "dedge":               0x01 << 29,
    "diss2g":              0x01 << 30,
    "diss2vs":             0x01 << 31
}
Fields["DRV_STATUS"] = {
    "otpw":                0x01,
    "ot":                  0x01 << 1,
    "s2ga":                0x01 << 2,
    "s2gb":                0x01 << 3,
    "s2vsa":               0x01 << 4,
    "s2vsb":               0x01 << 5,
    "ola":                 0x01 << 6,
    "olb":                 0x01 << 7,
    "t120":                0x01 << 8,
    "t143":                0x01 << 9,
    "t150":                0x01 << 10,
    "t157":                0x01 << 11,
    "CS_ACTUAL":           0x1f << 16,
    "stealth":             0x01 << 30,
    "stst":                0x01 << 31
}
Fields["PWMCONF"] = {
    "PWM_OFS":             0xff,
    "PWM_GRAD":            0xff << 8,
    "pwm_freq":            0x03 << 16,
    "pwm_autoscale":       0x01 << 18,
    "pwm_autograd":        0x01 << 19,
    "freewheel":           0x03 << 20,
    "PWM_REG":             0xf << 24,
    "PWM_LIM":             0xf << 28
}
Fields["PWM_SCALE"] = {
    "PWM_SCALE_SUM":       0xff,
    "PWM_SCALE_AUTO":      0x1ff << 16
}
Fields["PWM_AUTO"] = {
    "PWM_OFS_AUTO":        0xff,
    "PWM_GRAD_AUTO":       0xff << 16
}

FieldFormatters = dict(tmc2130.FieldFormatters)
FieldFormatters.update({
    "SEL_A":            (lambda v: "%d(%s)" % (v, ["TMC222x", "TMC220x"][v])),
    "s2vsa":            (lambda v: "1(LowSideShort_A!)" if v else ""),
    "s2vsb":            (lambda v: "1(LowSideShort_B!)" if v else ""),
    "PWM_SCALE_AUTO":   (lambda v: tmc2130.decode_signed_int(v, 9))
})


######################################################################
# TMC2208 communication
######################################################################

# Generate a CRC8-ATM value for a bytearray
def calc_crc8(data):
    crc = 0
    for b in data:
        for i in range(8):
            if (crc >> 7) ^ (b & 0x01):
                crc = (crc << 1) ^ 0x07
            else:
                crc = (crc << 1)
            crc &= 0xff
            b >>= 1
    return crc

# Add serial start and stop bits to a message in a bytearray
def add_serial_bits(data):
    out = 0
    pos = 0
    for d in data:
        b = (d << 1) | 0x200
        out |= (b << pos)
        pos += 10
    res = bytearray()
    for i in range((pos+7)//8):
        res.append((out >> (i*8)) & 0xff)
    return res

# Generate a tmc2208 read register message
def encode_tmc2208_read(sync, addr, reg):
    msg = bytearray([sync, addr, reg])
    msg.append(calc_crc8(msg))
    return add_serial_bits(msg)

# Generate a tmc2208 write register message
def encode_tmc2208_write(sync, addr, reg, val):
    msg = bytearray([sync, addr, reg, (val >> 24) & 0xff, (val >> 16) & 0xff,
                     (val >> 8) & 0xff, val & 0xff])
    msg.append(calc_crc8(msg))
    return add_serial_bits(msg)

# Extract a tmc2208 read response message
def decode_tmc2208_read(reg, data):
    # Convert data into a long integer for easy manipulation
    if len(data) != 10:
        return None
    mval = pos = 0
    for d in bytearray(data):
        mval |= d << pos
        pos += 8
    # Extract register value
    val = ((((mval >> 31) & 0xff) << 24) | (((mval >> 41) & 0xff) << 16)
           | (((mval >> 51) & 0xff) << 8) | ((mval >> 61) & 0xff))
    # Verify start/stop bits and crc
    encoded_data = encode_tmc2208_write(0x05, 0xff, reg, val)
    if data != encoded_data:
        return None
    return val


######################################################################
# TMC2208 printer object
######################################################################

class TMC2208:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        # pin setup
        ppins = self.printer.lookup_object("pins")
        rx_pin_params = ppins.lookup_pin(
            config.get('uart_pin'), can_pullup=True)
        tx_pin_desc = config.get('tx_pin', None)
        if tx_pin_desc is None:
            tx_pin_params = rx_pin_params
        else:
            tx_pin_params = ppins.lookup_pin(tx_pin_desc)
        if rx_pin_params['chip'] is not tx_pin_params['chip']:
            raise ppins.error("TMC2208 rx and tx pins must be on the same mcu")
        self.mcu = rx_pin_params['chip']
        self.pullup = rx_pin_params['pullup']
        self.rx_pin = rx_pin_params['pin']
        self.tx_pin = tx_pin_params['pin']
        self.oid = self.mcu.create_oid()
        self.tmcuart_send_cmd = None
        self.mcu.register_config_callback(self.build_config)
        # Add DUMP_TMC command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "DUMP_TMC", "STEPPER", self.name,
            self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)
        # Setup basic register values
        self.ifcnt = None
        self.regs = collections.OrderedDict()
        self.fields = tmc2130.FieldHelper(Fields, FieldFormatters, self.regs)
        self.fields.set_field("pdn_disable", True)
        self.fields.set_field("mstep_reg_select", True)
        self.fields.set_field("multistep_filt", True)
        vsense, irun, ihold = tmc2130.get_config_current(config)
        self.fields.set_field("vsense", vsense)
        self.fields.set_field("IHOLD", ihold)
        self.fields.set_field("IRUN", irun)
        mres, en, thresh = tmc2130.get_config_stealthchop(config, TMC_FREQUENCY)
        self.fields.set_field("MRES", mres)
        self.fields.set_field("en_spreadCycle", not en)
        self.fields.set_field("TPWMTHRS", thresh)
        # Allow other registers to be set from the config
        set_config_field = self.fields.set_config_field
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 0)
        set_config_field(config, "TBL", 2, "driver_BLANK_TIME_SELECT")
        set_config_field(config, "intpol", True, "interpolate")
        set_config_field(config, "IHOLDDELAY", 8)
        set_config_field(config, "TPOWERDOWN", 20)
        set_config_field(config, "PWM_OFS", 36)
        set_config_field(config, "PWM_GRAD", 14)
        set_config_field(config, "pwm_freq", 1)
        set_config_field(config, "pwm_autoscale", True)
        set_config_field(config, "pwm_autograd", True)
        set_config_field(config, "PWM_REG", 8)
        set_config_field(config, "PWM_LIM", 12)
    def build_config(self):
        bit_ticks = int(self.mcu.get_adjusted_freq() / 9000.)
        self.mcu.add_config_cmd(
            "config_tmcuart oid=%d rx_pin=%s pull_up=%d tx_pin=%s bit_time=%d"
            % (self.oid, self.rx_pin, self.pullup, self.tx_pin, bit_ticks))
        cmd_queue = self.mcu.alloc_command_queue()
        self.tmcuart_send_cmd = self.mcu.lookup_command(
            "tmcuart_send oid=%c write=%*s read=%c", cq=cmd_queue)
    def handle_connect(self):
        for reg_name, val in self.regs.items():
            self.set_register(reg_name, val)
    def get_register(self, reg_name):
        reg = Registers[reg_name]
        msg = encode_tmc2208_read(0xf5, 0x00, reg)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return 0
        for retry in range(5):
            params = self.tmcuart_send_cmd.send_with_response(
                [self.oid, msg, 10], 'tmcuart_response', self.oid)
            val = decode_tmc2208_read(reg, params['read'])
            if val is not None:
                return val
        raise self.printer.config_error(
            "Unable to read tmc2208 '%s' register %s" % (self.name, reg_name))
    def set_register(self, reg_name, val):
        msg = encode_tmc2208_write(0xf5, 0x00, Registers[reg_name] | 0x80, val)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        for retry in range(5):
            ifcnt = self.ifcnt
            if ifcnt is None:
                self.ifcnt = ifcnt = self.get_register("IFCNT")
            params = self.tmcuart_send_cmd.send_with_response(
                [self.oid, msg, 0], 'tmcuart_response', self.oid)
            self.ifcnt = self.get_register("IFCNT")
            if self.ifcnt == (ifcnt + 1) & 0xff:
                return
        raise self.printer.config_error(
            "Unable to write tmc2208 '%s' register %s" % (self.name, reg_name))
    def get_microsteps(self):
        return 256 >> self.fields.get_field("MRES")
    def get_phase(self):
        mscnt = self.fields.get_field("MSCNT", self.get_register("MSCNT"))
        return mscnt >> self.fields.get_field("MRES")
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, params):
        self.printer.lookup_object('toolhead').get_last_move_time()
        gcode = self.printer.lookup_object('gcode')
        logging.info("DUMP_TMC %s", self.name)
        for reg_name in ReadRegisters:
            try:
                val = self.get_register(reg_name)
            except self.printer.config_error as e:
                raise gcode.error(str(e))
            # IOIN has different mappings depending on the driver type
            # (SEL_A field of IOIN reg)
            if reg_name == "IOIN":
                drv_type = self.fields.get_field("SEL_A", val)
                reg_name = "IOIN@TMC220x" if drv_type else "IOIN@TMC222x"
            msg = self.fields.pretty_format(reg_name, val)
            logging.info(msg)
            gcode.respond_info(msg)

def load_config_prefix(config):
    return TMC2208(config)
