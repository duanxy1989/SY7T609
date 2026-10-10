import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import maybe_simple_id
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_POWER_FACTOR,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_POWER,
    CONF_REACTIVE_POWER,
    CONF_ENERGY,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_APPARENT_POWER,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_LIGHTBULB,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_VOLT_AMPS,
    UNIT_VOLT_AMPS_REACTIVE,
    UNIT_WATT_HOURS,
    UNIT_HERTZ,
    UNIT_CELSIUS,
)

# 常量定义
CONF_CHIP_TEMPERATURE = "chip_temperature"
# 反向有功电量（倒送累计），来自芯片 EPMcnt 寄存器
CONF_REVERSE_ENERGY = "reverse_energy"
# 视在功率（VA），来自芯片 VA 寄存器
CONF_APPARENT_POWER = "apparent_power"
# 线路频率（Hz），来自芯片 Frequency 寄存器
CONF_FREQUENCY = "frequency"
# 死区阈值：绝对值低于该值的近零读数按 0 处理，抑制空载噪声；
# 高于阈值的读数保留原始符号，仍可反映反向功率（倒送）。
CONF_POWER_DEADBAND = "power_deadband"
CONF_REACTIVE_POWER_DEADBAND = "reactive_power_deadband"
CONF_POWER_FACTOR_DEADBAND = "power_factor_deadband"

DEPENDENCIES = ["uart"]

# 定义命名空间和核心类
sy7t609_ns = cg.esphome_ns.namespace("sy7t609")
SY7T609 = sy7t609_ns.class_("SY7T609_UART", cg.PollingComponent, uart.UARTDevice)

# 定义动作
ResetEnergyAction = sy7t609_ns.class_("ResetEnergyAction", automation.Action)
ResetCalibrationAction = sy7t609_ns.class_("ResetCalibrationAction", automation.Action)

# 传感器键 -> setter 名称（新增读取数据时在此追加一行即可）
SENSOR_SETTERS = {
    CONF_POWER_FACTOR: "set_power_factor_sensor",
    CONF_VOLTAGE: "set_voltage_sensor",
    CONF_CURRENT: "set_current_sensor",
    CONF_POWER: "set_power_sensor",
    CONF_REACTIVE_POWER: "set_power_reactive_sensor",
    CONF_APPARENT_POWER: "set_apparent_power_sensor",
    CONF_FREQUENCY: "set_frequency_sensor",
    CONF_ENERGY: "set_energy_sensor",
    CONF_REVERSE_ENERGY: "set_reverse_energy_sensor",
    CONF_CHIP_TEMPERATURE: "set_temperature_sensor",
}

# 定义 Schema
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SY7T609),
            cv.Optional(CONF_POWER_DEADBAND, default=0.5): cv.float_,
            cv.Optional(CONF_REACTIVE_POWER_DEADBAND, default=0.5): cv.float_,
            cv.Optional(CONF_POWER_FACTOR_DEADBAND, default=0.05): cv.float_,
            cv.Optional(CONF_POWER_FACTOR): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_POWER_FACTOR,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_REACTIVE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE,
                icon=ICON_LIGHTBULB,
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_APPARENT_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_APPARENT_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_REVERSE_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_CHIP_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


# ----------------------------------------------------------------------------
# 无参动作（复用模式）：reset_energy / reset_calibration
# ----------------------------------------------------------------------------
def register_simple_action(action_name, action_type):
    @automation.register_action(
        action_name,
        action_type,
        maybe_simple_id(
            {
                cv.Required(CONF_ID): cv.use_id(SY7T609),
            }
        ),
        synchronous=False,  # 实际执行被推迟到 loop() 的回调队列中
    )
    async def _to_code(config, action_id, template_arg, args):
        parent = await cg.get_variable(config[CONF_ID])
        return cg.new_Pvariable(action_id, template_arg, parent)


register_simple_action("sy7t609.reset_energy", ResetEnergyAction)
register_simple_action("sy7t609.reset_calibration", ResetCalibrationAction)


# ----------------------------------------------------------------------------
# to_code 主逻辑
# ----------------------------------------------------------------------------
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # 死区阈值
    cg.add(var.set_power_deadband(config[CONF_POWER_DEADBAND]))
    cg.add(var.set_reactive_power_deadband(config[CONF_REACTIVE_POWER_DEADBAND]))
    cg.add(var.set_power_factor_deadband(config[CONF_POWER_FACTOR_DEADBAND]))

    # 传感器映射
    for key, setter in SENSOR_SETTERS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))
