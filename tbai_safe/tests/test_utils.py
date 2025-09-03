from tbai_safe.utils import remove_kwargs


# def test_rate_sleep():
#   rate = Rate(2)  # 10 Hz = 0.1s period
#   start = time.perf_counter()
#   rate.sleep()
#   elapsed = time.perf_counter() - start
#   assert 0.49 < elapsed < 0.51

#   time.sleep(0.25)
#   start = time.perf_counter()
#   rate.sleep()
#   elapsed = time.perf_counter() - start
#   assert 0.24 < elapsed < 0.26


def test_remove_kwargs():
  def example_func(a, b, d=4, c=3):
    return a * b / c**d

  def example_func_2(a, b, c=3, d=4):
    return example_func(a, d=d, c=c, b=b)

  fn, old_source, new_source = remove_kwargs(example_func_2, locals=locals(), globals=globals())
  assert new_source == "def example_func_2(a, b, c=3, d=4):\n    return example_func(a, b, d, c)"

  assert fn(1, 2, 3, 4) == 1 * 2 / 3**4
  assert fn(1, 2, d=4, c=3) == example_func(1, 2, 4, 3)
