import asyncio
from asyncio.streams import StreamReader, StreamWriter
from typing import Optional
import aioconsole


async def send_input(writer: StreamWriter, host: Optional[str] = None, port: Optional[int] = None):
    while str.lower(command := await aioconsole.ainput('>>> ')) != 'q':
        if command != '':
            if host != None and port != None:
                print(f'Sending "{command}" to {host}:{port}')
            writer.write(f'{command}\n'.encode('utf8'))
    print('Quitting')


async def print_reply(reader: StreamReader, host: Optional[str] = None, port: Optional[int] = None):
    async for message in reader:
        if host != None and port != None:
            print(f'Message from {host}:{port}: {message}')
        else:
            print(message)
    print('Disconnected')


async def main():
    host = '192.168.0.8'
    port = 8080
    reader, writer = await asyncio.open_connection(host=host, port=port)
    send_input_coro = send_input(writer, host, port)
    print_reply_coro = print_reply(reader, host, port)
    await asyncio.wait((send_input_coro, print_reply_coro), return_when=asyncio.FIRST_COMPLETED)


asyncio.run(main())
