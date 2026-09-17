import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    UNIT_KILOGRAM,
    DEVICE_CLASS_WEIGHT,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor"]

s7_mcu_ns = cg.esphome_ns.namespace("s7_mcu")
S7McuComponent = s7_mcu_ns.class_(
    "S7McuComponent", cg.Component, uart.UARTDevice
)

CONF_WEIGHT_HI = "weight_hi"
CONF_WEIGHT_LO = "weight_lo"
CONF_WEIGHT_DIV = "weight_div"
CONF_WEIGHT_OFFSET = "weight_offset"
CONF_WEIGHT_SENSOR = "weight"
CONF_MCU_ALIVE = "mcu_alive"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(S7McuComponent),
            cv.Optional(CONF_WEIGHT_HI, default=0): cv.int_range(min=0, max=30),
            cv.Optional(CONF_WEIGHT_LO, default=1): cv.int_range(min=0, max=30),
            cv.Optional(CONF_WEIGHT_DIV, default=100.0): cv.float_,
            cv.Optional(CONF_WEIGHT_OFFSET, default=0.0): cv.float_,
            cv.Optional(CONF_WEIGHT_SENSOR): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOGRAM,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_WEIGHT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MCU_ALIVE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_weight_hi(config[CONF_WEIGHT_HI]))
    cg.add(var.set_weight_lo(config[CONF_WEIGHT_LO]))
    cg.add(var.set_weight_div(config[CONF_WEIGHT_DIV]))
    cg.add(var.set_weight_offset(config[CONF_WEIGHT_OFFSET]))

    if CONF_WEIGHT_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_WEIGHT_SENSOR])
        cg.add(var.set_weight_sensor(sens))

    if CONF_MCU_ALIVE in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_MCU_ALIVE])
        cg.add(var.set_mcu_alive_sensor(bs))
