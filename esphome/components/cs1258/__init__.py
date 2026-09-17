import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome import pins
from esphome.const import (
    CONF_ID,
    UNIT_OHM,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

CS1258_ns = cg.esphome_ns.namespace("cs1258")
CS1258Component = CS1258_ns.class_("CS1258Component", cg.Component)

CONF_PIN_CS = "pin_cs"
CONF_PIN_CLK = "pin_clk"
CONF_PIN_DIO = "pin_dio"
CONF_Z_SCALE = "z_scale"
CONF_Z_OFFSET = "z_offset"
CONF_CAL_OFFSET = "cal_offset"
CONF_IMPEDANCE = "impedance"
CONF_CS_OK = "cs_ok"
CONF_BODY_RES = "body_res"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CS1258Component),
        cv.Optional(CONF_PIN_CS, default=16): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_PIN_CLK, default=14): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_PIN_DIO, default=12): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_Z_SCALE, default=1.0): cv.float_,
        cv.Optional(CONF_Z_OFFSET, default=0.0): cv.float_,
        cv.Optional(CONF_CAL_OFFSET, default=1100): cv.uint16_t,
        cv.Optional(CONF_IMPEDANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_OHM,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:flash-triangle",
        ),
        cv.Optional(CONF_BODY_RES): sensor.sensor_schema(
            unit_of_measurement=UNIT_OHM,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:resistor",
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_pin_cs(config[CONF_PIN_CS]))
    cg.add(var.set_pin_clk(config[CONF_PIN_CLK]))
    cg.add(var.set_pin_dio(config[CONF_PIN_DIO]))
    cg.add(var.set_z_scale(config[CONF_Z_SCALE]))
    cg.add(var.set_z_offset(config[CONF_Z_OFFSET]))
    cg.add(var.set_cal_offset(config[CONF_CAL_OFFSET]))

    if CONF_IMPEDANCE in config:
        sens = await sensor.new_sensor(config[CONF_IMPEDANCE])
        cg.add(var.set_impedance_sensor(sens))

    if CONF_BODY_RES in config:
        sens = await sensor.new_sensor(config[CONF_BODY_RES])
        cg.add(var.set_body_res_sensor(sens))
