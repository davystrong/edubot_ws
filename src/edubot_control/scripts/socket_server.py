import asyncio
from asyncio.streams import StreamReader, StreamWriter
from typing import Dict
import aioconsole
from uuid import uuid4

writers: Dict[str, StreamWriter] = {}


async def client_connected_cb(reader: StreamReader, writer: StreamWriter):
    print('Client connected')
    uuid = str(uuid4())
    writers[uuid] = writer
    async for message in reader:
        print(str(message[:-1], 'utf8'))
    del writers[uuid]
    print('Client disconnected')


async def main():
    await asyncio.start_server(client_connected_cb, host='0.0.0.0', port=8080)
    print('Use q command to quit')
    while str.lower(command := await aioconsole.ainput('>>> ')) != 'q':
        cmd_bytes = f'{command}\n'.encode('utf8')
        for writer in writers.values():
            writer.write(cmd_bytes)

asyncio.run(main())
