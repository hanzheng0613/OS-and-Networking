from bcc import BPF
import os
import time
from datetime import datetime, timedelta

bpf_text = """
int trace_write(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: FastDDS::write() called\\n", ts);
    return 0;
}
int trace_read(struct pt_regs *ctx) {
    u64 ts = bpf_ktime_get_ns() / 1000000;
    bpf_trace_printk("%llu ms: FastDDS::take_next_sample() called\\n", ts);
    return 0;
}
"""

b = BPF(text=bpf_text)



# Attach uprobes to your functions
b.attach_uprobe(name="/opt/ros/humble/lib/libfastrtps.so.2.6.10",sym="_ZN8eprosima7fastdds3dds14DataWriterImpl5writeEPv", fn_name="trace_write")
b.attach_uprobe(name="/opt/ros/humble/lib/libfastrtps.so.2.6.10",sym="_ZN8eprosima7fastdds3dds14DataReaderImpl12read_or_takeERNS1_18LoanableCollectionERNS1_16LoanableSequenceINS1_10SampleInfoESt17integral_constantIbLb1EEEEiRKNS_8fastrtps4rtps16InstanceHandle_tEtttbbb", fn_name="trace_read")

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

