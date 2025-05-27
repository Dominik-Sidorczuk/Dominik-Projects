from . import sensor

CONFIG_SCHEMA = sensor.CONFIG_SCHEMA

async def to_code(config):
    await sensor.to_code(config)