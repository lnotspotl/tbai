import time
from tbai_safe.utils import Rate


def test_rate_sleep():
  rate = Rate(2)  # 10 Hz = 0.1s period
  start = time.perf_counter()
  rate.sleep()
  elapsed = time.perf_counter() - start
  assert 0.49 < elapsed < 0.51

  time.sleep(0.25)
  start = time.perf_counter()
  rate.sleep()
  elapsed = time.perf_counter() - start
  assert 0.24 < elapsed < 0.26
