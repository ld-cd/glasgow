# glasgow -v script repro2.py analyzer2 -V A=3.3,B=3.3 BDAT=B0:7

import asyncio

await analyzer_iface.set_sampling_rate(48e6)
await analyzer_iface.set_prolog_size(int(200))
await analyzer_iface.set_epilog_size(int(800))


task = asyncio.create_task(analyzer_iface._monitor_queue_size())

# Wait a full prolog before starting
await asyncio.sleep(0.5)
for i in range(32):
    await analyzer_iface.force_trigger()
    # await asyncio.sleep(500 / 48e6)
    await analyzer_iface.interrupt()
    await asyncio.sleep(0.5)

    block = await analyzer_iface.read_sample_block()
    print(i, len(block.samples))
    block = await analyzer_iface.read_sample_block()
    print(i, len(block.samples))
        

await task
