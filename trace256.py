from bcc import BPF
import os
import time
from datetime import datetime, timedelta
import struct
import ctypes
bpf_text = """
#include <uapi/linux/ptrace.h>
#define MAX_MSG 256

struct data_t {
    u32 pid;
    u32 len;
    char msg[256];
};
struct buffer_t {
    void *payload;
    u32 size;
};

BPF_PERF_OUTPUT(events);

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




int trace_message(struct pt_regs *ctx) {
    
    struct data_t data = {};
    data.pid = bpf_get_current_pid_tgid() >> 32;

    u32 len = (u32)PT_REGS_PARM2(ctx); // user-space length
    if (len > 256)
        len = 256;

    data.len = len;

    // Always read MAX_MSG bytes, slice later in userspace
    bpf_probe_read_user(&data.msg, MAX_MSG, (void *)PT_REGS_PARM1(ctx));

    events.perf_submit(ctx, &data, sizeof(data));
    return 0;
}

"""

b = BPF(text=bpf_text)



b.attach_uprobe(name="/opt/ros/humble/lib/libfastrtps.so",sym="_ZN8eprosima7fastdds4rtps18SharedMemTransport4sendEPKhjPNS1_16LocatorsIteratorES6_RKNSt6chrono10time_pointINS7_3_V212steady_clockENS7_8durationIlSt5ratioILl1ELl1000000000EEEEEE", fn_name="trace_write")
b.attach_uprobe(name="/opt/ros/humble/lib/libfastrtps.so",sym="_ZN8eprosima7fastdds4rtps18SharedMemTransport12push_discardERKSt10shared_ptrINS1_16SharedMemManager6BufferEERKNS_8fastrtps4rtps9Locator_tE", fn_name="trace_read")

b.attach_uprobe(name="/opt/ros/humble/lib/libfastrtps.so", sym="_ZN8eprosima7fastdds4rtps18SharedMemTransport21copy_to_shared_bufferEPKhjRKNSt6chrono10time_pointINS5_3_V212steady_clockENS5_8durationIlSt5ratioILl1ELl1000000000EEEEEE", fn_name="trace_message")


print("Tracing... Ctrl-C to exit.")
boot_time = time.time() - (b.ksymtime / 1e6 if hasattr(b, "ksymtime") else 0)
class Data(ctypes.Structure):
    _fields_ = [
        ("pid", ctypes.c_uint32),
        ("len", ctypes.c_uint32),
        ("msg", ctypes.c_ubyte * 256)
    ]

def print_event(cpu, data, size):
    event = ctypes.cast(data, ctypes.POINTER(Data)).contents
    payload = bytes(event.msg[:event.len])
    print(f"PID={event.pid}, payload length={event.len}")
    print(f"Payload (hex): {payload.hex()}")

b["events"].open_perf_buffer(print_event)


while True:
    try:
        (task, pid, cpu, flags, ts, msg) = b.trace_fields()

        wall_time = datetime.fromtimestamp(time.time())
        ms = int(ts % 1000)
        print(f"{wall_time.strftime('%d:%m:%Y %H:%M:%S')}.{ms:03d} - {msg}")
        b.perf_buffer_poll()
    except KeyboardInterrupt:
        break

