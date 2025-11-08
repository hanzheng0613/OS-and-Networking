from bcc import BPF
import os
import time
from datetime import datetime, timedelta

bpf_text = """
int trace_write(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: rmw_publish() called\\n", ts);
    return 0;
}
int trace_read(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: rmw_take() called\\n", ts);
    return 0;
}
"""

b = BPF(text=bpf_text)



# Attach uprobes to your functions
b.attach_uprobe(name="/opt/ros/humble/lib/librmw_fastrtps_shared_cpp.so",sym="_ZN23rmw_fastrtps_shared_cpp13__rmw_publishEPKcPK15rmw_publisher_sPKvP26rmw_publisher_allocation_s", fn_name="trace_write")
b.attach_uprobe(name="/opt/ros/humble/lib/librmw_fastrtps_shared_cpp.so",sym="_ZN23rmw_fastrtps_shared_cpp20__rmw_take_with_infoEPKcPK18rmw_subscription_sPvPbP18rmw_message_info_sP29rmw_subscription_allocation_s", fn_name="trace_read")

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

