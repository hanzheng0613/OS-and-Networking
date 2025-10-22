from bcc import BPF
import time
from datetime import datetime, timedelta

bpf_text = """
int trace_write(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: publish called\\n", ts);
    return 0;
}
int trace_read(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: subscribe called\\n", ts);
    return 0;
}
"""

b = BPF(text=bpf_text)
binary = "/usr/local/lib/libfastdds.so"


# Attach uprobes to your functions
b.attach_uprobe(name="./publish", sym="publish", fn_name="trace_write")
b.attach_uprobe(name="/usr/local/lib/libfastdds.so",sym="_ZN8eprosima7fastdds3dds14DataReaderImpl16take_next_sampleEPvPNS1_10SampleInfoE", fn_name="trace_read")

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

