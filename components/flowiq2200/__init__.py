"""Kamstrup FlowIQ 2200 water meter over wM-Bus (CC1101 radio).

Ported from https://github.com/erikxson/watermeter-flowiq2200 (GPLv3).
Field definitions follow the wmbusmeters "kamwater" driver (GPLv3).
"""

import esphome.codegen as cg
from esphome import pins
from esphome.components import binary_sensor, sensor, spi, text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

CONF_GDO0_PIN = "gdo0_pin"
CONF_METER_ID = "meter_id"
CONF_KEY = "key"
CONF_RSSI = "rssi"

flowiq2200_ns = cg.esphome_ns.namespace("flowiq2200")
FlowIQ2200Component = flowiq2200_ns.class_(
    "FlowIQ2200Component", cg.Component, spi.SPIDevice
)


def _hex_string(num_bytes, what):
    def validator(value):
        if not isinstance(value, str):
            raise cv.Invalid(
                f"{what} must be a quoted string (e.g. \"0123...\"); unquoted numbers lose leading zeros"
            )
        cleaned = value.strip().replace(" ", "").replace(":", "").replace("-", "")
        if cleaned.lower().startswith("0x"):
            cleaned = cleaned[2:]
        if len(cleaned) != num_bytes * 2:
            raise cv.Invalid(
                f"{what} must be exactly {num_bytes * 2} hex digits, got {len(cleaned)}"
            )
        try:
            bytes.fromhex(cleaned)
        except ValueError as err:
            raise cv.Invalid(f"{what} contains non-hex characters") from err
        return cleaned.upper()

    return validator


def _volume(state_class=cv.UNDEFINED):
    return sensor.sensor_schema(
        unit_of_measurement="m³",
        icon="mdi:water",
        accuracy_decimals=3,
        device_class="water",
        state_class=state_class,
    )


def _flow():
    return sensor.sensor_schema(
        unit_of_measurement="L/h",
        icon="mdi:water-pump",
        accuracy_decimals=0,
        device_class="volume_flow_rate",
        state_class="measurement",
    )


def _temperature():
    return sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=0,
        device_class="temperature",
        state_class="measurement",
    )


def _text(icon):
    return text_sensor.text_sensor_schema(icon=icon)


def _diag_text(icon):
    return text_sensor.text_sensor_schema(icon=icon, entity_category="diagnostic")


def _problem(device_class="problem"):
    return binary_sensor.binary_sensor_schema(device_class=device_class)


# key: (index into the C++ enum, schema). Order of indices must match flowiq_decoder.h / flowiq2200.h.
NUMERIC_SENSORS = {
    "volume": (0, _volume("total_increasing")),       # total_m3
    "month_start": (1, _volume()),                     # target_m3 (reading at the target/billing date)
    "flow": (2, _flow()),                              # flow_m3h
    "max_flow_last_day": (3, _flow()),
    "min_flow_last_day": (4, _flow()),
    "min_water_temperature_last_day": (5, _temperature()),
    "max_water_temperature_last_day": (6, _temperature()),
    "min_ambient_temperature_last_day": (7, _temperature()),
    "max_ambient_temperature_last_day": (8, _temperature()),
}

TEXT_SENSORS = {
    "status": (0, _text("mdi:information-outline")),
    "target_date": (1, _text("mdi:calendar")),
    "time_dry": (2, _diag_text("mdi:timer-sand")),
    "time_reversed": (3, _diag_text("mdi:timer-sand")),
    "time_leaking": (4, _diag_text("mdi:timer-sand")),
    "time_bursting": (5, _diag_text("mdi:timer-sand")),
    "time_ambient_temperature": (6, _diag_text("mdi:timer-sand")),
    "time_flow_above_q4": (7, _diag_text("mdi:timer-sand")),
    "acoustic_noise": (8, _diag_text("mdi:waveform")),
}

BINARY_SENSORS = {
    "dry": (0, _problem()),
    "reverse": (1, _problem()),
    "leak": (2, _problem("moisture")),
    "burst": (3, _problem()),
    "tamper": (4, _problem("tamper")),
    "low_battery": (5, _problem("battery")),
    "ambient_temperature_alarm": (6, _problem()),
    "flow_above_q4": (7, _problem()),
    "no_consumption": (8, binary_sensor.binary_sensor_schema()),
}

_schema = {
    cv.GenerateID(): cv.declare_id(FlowIQ2200Component),
    cv.Required(CONF_GDO0_PIN): pins.internal_gpio_input_pin_schema,
    cv.Required(CONF_METER_ID): _hex_string(4, "meter_id"),
    cv.Required(CONF_KEY): _hex_string(16, "key"),
    cv.Optional(CONF_RSSI): sensor.sensor_schema(
        unit_of_measurement="dBm",
        accuracy_decimals=0,
        device_class="signal_strength",
        state_class="measurement",
        entity_category="diagnostic",
    ),
    # Removed: flow is now read as the full 16-bit value from every frame.
    cv.Optional("flow_full"): cv.invalid(
        "flow_full was removed: 'flow' now carries the exact value from every frame"
    ),
}
for table in (NUMERIC_SENSORS, TEXT_SENSORS, BINARY_SENSORS):
    for key, (_, schema) in table.items():
        _schema[cv.Optional(key)] = schema

CONFIG_SCHEMA = (
    cv.Schema(_schema)
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(pin))
    cg.add(var.set_meter_id(int(config[CONF_METER_ID], 16)))
    cg.add(var.set_key(config[CONF_KEY]))

    if CONF_RSSI in config:
        sens = await sensor.new_sensor(config[CONF_RSSI])
        cg.add(var.set_rssi_sensor(sens))

    for key, (idx, _) in NUMERIC_SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(var.set_numeric_sensor(idx, sens))
    for key, (idx, _) in TEXT_SENSORS.items():
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(var.set_text_sensor(idx, sens))
    for key, (idx, _) in BINARY_SENSORS.items():
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(var.set_binary_sensor(idx, sens))
