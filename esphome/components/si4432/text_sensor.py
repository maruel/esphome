from esphome import pins
from esphome.components import text_sensor
from esphome.components import spi
import esphome.codegen as cg
import esphome.config_validation as cv

DEPENDENCIES = ["spi"]
# AUTO_LOAD = ["text_sensor"]
# MULTI_CONF = True

CONF_IRQ_PIN = "irq_pin"
CONF_SDN_PIN = "sdn_pin"

# TODO(maruel): Frequency. Validate 240 <= f <= 930.
# TODO(maruel): Baud. Validate 1000 <= b <= 256000.
# TODO(maruel): Channel.

si4432_ns = cg.esphome_ns.namespace("si4432")
Si4432Component = si4432_ns.class_(
    "Si4432Component", text_sensor.TextSensor, cg.Component, spi.SPIDevice
)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema()
    .extend(
        {
            cv.GenerateID(): cv.declare_id(Si4432Component),
            cv.Required(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_SDN_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(spi.spi_device_schema(cs_pin_required=True))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
    cg.add(var.set_irq_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_SDN_PIN])
    cg.add(var.set_sdn_pin(pin))
