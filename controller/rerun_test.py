import time

import rerun as rr

rr.init("counter")
rr.connect_grpc()

count = 0
while True:
    rr.log(
        "counter",
        rr.TextDocument(f"# Count: {count}", media_type=rr.MediaType.MARKDOWN),
    )
    rr.log("count", rr.Scalars(count))
    time.sleep(1)
    count += 1
