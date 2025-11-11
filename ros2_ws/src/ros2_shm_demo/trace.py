from bcc import BPF
import os
import time
from datetime import datetime, timedelta

bpf_text = """
int trace_write(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: SharedMemTransport::send() called\\n", ts);
    return 0;
}
int trace_read(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: SharedMemTransport::push_discard() called\\n", ts);
    return 0;
}
"""

b = BPF(text=bpf_text)



b.attach_uprobe(name="/opt/ros/humble/lib/libfastrtps.so",sym="_ZN8eprosima7fastdds4rtps18SharedMemTransport4sendEPKhjPNS1_16LocatorsIteratorES6_RKNSt6chrono10time_pointINS7_3_V212steady_clockENS7_8durationIlSt5ratioILl1ELl1000000000EEEEEE", fn_name="trace_write")
b.attach_uprobe(name="/opt/ros/humble/lib/libfastrtps.so",sym="_ZN8eprosima7fastdds4rtps18SharedMemTransport12push_discardERKSt10shared_ptrINS1_16SharedMemManager6BufferEERKNS_8fastrtps4rtps9Locator_tE", fn_name="trace_read")


print("Tracing... Ctrl-C to exit.")
boot_time = time.time() - (b.ksymtime / 1e6 if hasattr(b, "ksymtime") else 0)

while True:
    try:
        (task, pid, cpu, flags, ts, msg) = b.trace_fields()
        # ts = monotonic ms since boot
        # Convert to wall-clock time
        wall_time = datetime.fromtimestamp(time.time())
        ms = int(ts % 1000)
        print(f"{wall_time.strftime('%d:%m:%Y %H:%M:%S')}.{ms:03d} - {msg}")
    except KeyboardInterrupt:
        break

