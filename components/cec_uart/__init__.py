import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@JosVanEijndhoven"]

cec_uart_ns = cg.esphome_ns.namespace("cec_uart")

CecUart = cec_uart_ns.class_(
    "CecUart", cg.Component, uart.UartDevice
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(CecUart),
    }
).extend(uart.UART_DEVICE_SCHEMA)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "cec_uart", require_tx=True
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cec_pin_ = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_pin(cec_pin_))

