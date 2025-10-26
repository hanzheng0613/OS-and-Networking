#!/usr/bin/env python3

from bcc import BPF


from datetime import datetime

bpf_text = """
int trace_publish(void *ctx) {
    bpf_trace_printk("Publisher: publish_message() called\\n");
    return 0;
}

int trace_take(void *ctx) {
    bpf_trace_printk("Subscriber: handle_message() called\\n");
    return 0;
}
"""

b = BPF(text=bpf_text)


publisher_bin = "/home/hanzhengwang/Desktop/OS/ros2_ws/build/ros2_shm_demo/shm_publisher"


subscriber_bin = "/home/hanzhengwang/Desktop/OS/ros2_ws/build/ros2_shm_demo/shm_subscriber"




b.attach_uprobe(name=publisher_bin,
                sym="_ZN12ShmPublisher15publish_messageEv",
                fn_name="trace_publish")

b.attach_uprobe(name=subscriber_bin,
                sym="_ZN6rclcpp12SubscriptionIN8std_msgs3msg7String_ISaIvEEES4_S5_S5_NS_23message_memory_strategy21MessageMemoryStrategyIS5_S4_EEE14handle_messageERSt10shared_ptrIvERKNS_11MessageInfoE",
                fn_name="trace_take")
                
                
print("Start Trace:")

try:
    while True:
        (task, pid, cpu, flags, ts, msg) = b.trace_fields()

        now = datetime.now()
        
        timestamp = now.strftime("%H:%M:%S.%f")[:-3] 
        
        
        print(f"{timestamp} {msg.decode('utf-8')}")
except KeyboardInterrupt:
    print("Detaching...")

