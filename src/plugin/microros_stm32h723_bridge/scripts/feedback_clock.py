"""Clock admission for feedback; never substitutes receipt time for source time."""


class FeedbackClock:
    def __init__(self):
        self.state = 'waiting'
        self.message = '等待反馈时钟'
        self.generation = 0
        self.last_received = None
        self.offset_ms = None
        self.last_stamp = None
        self.valid_since = None
        self.previous = None
        self.firmware_at = None
        self.firmware_ok = True
        self.firmware_epoch = None

    def invalidate(self, message, state='unsynchronized'):
        if self.state == 'synchronized':
            self.generation += 1
        self.state, self.message = state, message
        self.valid_since = None

    def firmware(self, data, steady_ns):
        if len(data) == 16 and data[0] == 1:
            return  # Older firmware: admission still checks actual source timestamps.
        if len(data) != 24 or data[0] != 2:
            self.firmware_ok = False
            self.firmware_at = steady_ns
            self.invalidate('固件时间诊断格式无效')
            return
        epoch = data[22]
        if self.firmware_epoch is not None and epoch != self.firmware_epoch:
            self.invalidate('固件时间基准改变，等待稳定反馈')
            self.last_stamp = None
            self.generation += 1
        self.firmware_epoch = epoch
        self.firmware_at = steady_ns
        self.firmware_ok = data[16] == 1 and data[17] < 15000
        if not self.firmware_ok:
            self.invalidate('固件时间尚未同步或已失步')

    def refresh(self, steady_ns):
        if self.last_received is not None and steady_ns - self.last_received >= 1000000000:
            self.invalidate('反馈通信已过期', 'stale')
        elif self.firmware_at is not None and steady_ns - self.firmware_at >= 3000000000:
            self.invalidate('固件时间诊断已过期', 'stale')

    def observe(self, stamp_ns, now_ns, steady_ns):
        self.refresh(steady_ns)
        if self.previous is not None:
            ros, steady = self.previous
            if abs((now_ns - ros) - (steady_ns - steady)) > 250000000:
                self.invalidate('Jetson ROS 时间发生跳变')
                self.last_stamp = None
                self.generation += 1
        self.previous = now_ns, steady_ns
        self.last_received = steady_ns
        self.offset_ms = (stamp_ns - now_ns) / 1000000
        if not self.firmware_ok or (self.firmware_at is not None and steady_ns-self.firmware_at >= 3000000000):
            self.invalidate('固件时间尚未同步或诊断已过期')
            return False
        if stamp_ns <= 0 or stamp_ns - now_ns > 50000000 or now_ns - stamp_ns >= 1000000000:
            self.invalidate('反馈时间戳无效、超前或过旧')
            return False
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            if self.last_stamp - stamp_ns > 50000000:
                self.invalidate('反馈时间戳回拨，等待新时间基准稳定')
                self.last_stamp = stamp_ns
                self.generation += 1
            return False
        self.last_stamp = stamp_ns
        if self.valid_since is None:
            self.valid_since = steady_ns
        if steady_ns - self.valid_since < 200000000:
            self.state, self.message = 'unsynchronized', '等待时间戳连续稳定 200 ms'
            return False
        self.state, self.message = 'synchronized', '反馈时钟已同步'
        return True
