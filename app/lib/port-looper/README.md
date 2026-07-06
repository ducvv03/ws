# worker/looper

Standalone **C++11** port of `android::Looper` (originally `libutils/Looper.cpp`),
an epoll-based event loop supporting fd callbacks and delayed messages.

All AOSP/libutils dependencies were stripped out:

| Original (libutils)        | Port (`worker`)                          |
| -------------------------- | ---------------------------------------- |
| `sp<>` / `wp<>` / `RefBase`| `std::shared_ptr` / `std::weak_ptr`      |
| `Vector`                   | `std::vector`                            |
| `Mutex` / `AutoMutex`      | `std::mutex` / `std::lock_guard`         |
| `Timers` (`nsecs_t`, `systemTime`) | `std::chrono::steady_clock` (ns as `int64_t`) |
| `android::base::unique_fd` | `worker::unique_fd` (minimal RAII)       |
| `ALOG*` / `LOG_ALWAYS_FATAL*` | internal `stderr` logging in `Looper.cpp` |

Platform requirement: Linux (`epoll`, `eventfd`). No external libraries beyond
the C++ standard library and pthreads.

## Build

```sh
cmake -B build && cmake --build build
./build/looper_demo
```

Or directly:

```sh
g++ -std=c++11 -Iinclude Looper.cpp looper_demo.cpp -o looper_demo -lpthread
```

## Layout

- `include/looper/Looper.h` — public API (namespace `worker`)
- `Looper.cpp` — implementation
- `looper_demo.cpp` — smoke test covering fd callbacks, delayed messages, cross-thread `wake()`
