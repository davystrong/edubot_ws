# For simulating the control response of the robot
import asyncio
from asyncio.streams import StreamReader, StreamWriter
from typing import Dict
import aioconsole
from uuid import uuid4
from math import isclose

step_seconds = 0.1
pulses_per_revolution = 1496.0

# The relationship between voltage and angular acceleration (pulses/s²)
alpha = 10
# The relationship between voltage and angular speed (pulses/s)
beta = 10

writers: Dict[str, StreamWriter] = {}

# This should be int, but floats are better for simulation
pulses_state = [0.0, 0.0]
speed_state = [0.0, 0.0]
voltage_state = [0.0, 0.0]


async def command_handler(reader: StreamReader):
    global voltage_state
    async for message in reader:
        message = str(message, 'utf8')
        if message[0] == 'w':
            message = message[1:].strip()
            voltage_state = [float(f) for f in message.split(':')]
            if not isclose(voltage_state[0], 0, abs_tol=1e-5) or not isclose(voltage_state[1], 0, abs_tol=1e-5):
                print(voltage_state)


async def update_state():
    for index, voltage in enumerate(voltage_state):
        pulses_state[index] += step_seconds*speed_state[index]
        speed_state[index] += step_seconds * \
            (voltage*beta - speed_state[index])*alpha


async def send_state(writer: StreamWriter):
    """
    This only sends the pulses and period, nothing else
    """
    writer.write(
        f'{step_seconds},{",".join([str(x) for x in pulses_state])},0,0,0,0,0,0\n'.encode('utf8'))


async def state_handler(writer: StreamWriter):
    while not writer.is_closing():
        await asyncio.wait([
            update_state(),
            send_state(writer),
            asyncio.sleep(step_seconds)
        ])


async def client_connected_cb(reader: StreamReader, writer: StreamWriter):
    print('Client connected')
    uuid = str(uuid4())
    writers[uuid] = writer
    await asyncio.wait([state_handler(writer), command_handler(reader)])
    del writers[uuid]
    print('Client disconnected')


async def main():
    await asyncio.start_server(client_connected_cb, host='0.0.0.0', port=8080)
    print('Use q command to quit')
    while str.lower(command := await aioconsole.ainput('>>> ')) != 'q':
        # cmd_bytes = f'{command}\n'.encode('utf8')
        # for writer in writers.values():
        #     writer.write(cmd_bytes)
        pass

asyncio.run(main())
