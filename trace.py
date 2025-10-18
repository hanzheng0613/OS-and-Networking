from bcc import BPF
import time

binary_path = "./writer"  # change to your binary

bpf_program = """
#include <uapi/linux/ptrace.h>

int trace_write(struct pt_regs *ctx) {
    bpf_trace_printk("write() called\\n");
    return 0;
}

int trace_read(struct pt_regs *ctx) {
    bpf_trace_printk("read() called\\n");
    return 0;
}
"""

b = BPF(text=bpf_program)
b.attach_uprobe(name=binary_path, sym="write", fn_name="trace_write")
b.attach_uprobe(name=binary_path, sym="read", fn_name="trace_read")

print("Tracing write() and read() timestamps... Ctrl+C to exit.")

# Print with real-time timestamps
for line in b.trace_readline():
    
    ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + f".{int(time.time()*1000)%1000:03d}"
    print(f"{ts} {line.decode('utf-8').strip()}")
