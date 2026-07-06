/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Standalone, framework-free C++11 port of android::Looper (from libutils).
//
// All AOSP/libutils dependencies have been removed:
//   sp<>/wp<>/RefBase   -> std::shared_ptr / std::weak_ptr
//   Vector              -> std::vector
//   Mutex/AutoMutex     -> std::mutex / std::lock_guard
//   Timers (nsecs_t)    -> std::chrono::steady_clock (nanoseconds as int64_t)
//   android::base::unique_fd -> worker::unique_fd (below)
//   ALOG*/LOG_*         -> internal logging in Looper.cpp
//
// It builds on its own with any C++11 compiler on Linux (epoll/eventfd).

#ifndef WORKER_LOOPER_H
#define WORKER_LOOPER_H

#include <sys/epoll.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace worker {

// Monotonic time in nanoseconds, replacing libutils' nsecs_t.
using nsecs_t = int64_t;

/**
 * Minimal move-only RAII wrapper around a file descriptor, replacing
 * android::base::unique_fd.
 */
class unique_fd {
public:
    unique_fd() = default;
    explicit unique_fd(int fd) : mFd(fd) {}
    ~unique_fd() { reset(); }

    unique_fd(unique_fd&& other) noexcept : mFd(other.mFd) { other.mFd = -1; }
    unique_fd& operator=(unique_fd&& other) noexcept {
        if (this != &other) {
            reset(other.mFd);
            other.mFd = -1;
        }
        return *this;
    }

    unique_fd(const unique_fd&) = delete;
    unique_fd& operator=(const unique_fd&) = delete;

    int get() const { return mFd; }
    void reset(int newFd = -1);
    int release() {
        int fd = mFd;
        mFd = -1;
        return fd;
    }

    bool ok() const { return mFd >= 0; }

private:
    int mFd = -1;
};

/*
 * NOTE: The Looper enums and the signature of Looper_callbackFunc are kept
 * identical to the original so existing event-loop code ports unchanged.
 */

/**
 * For callback-based event loops, this is the prototype of the function
 * that is called when a file descriptor event occurs.
 *
 * Implementations should return 1 to continue receiving callbacks, or 0
 * to have this file descriptor and callback unregistered from the looper.
 */
typedef int (*Looper_callbackFunc)(int fd, int events, void* data);

/**
 * A message that can be posted to a Looper.
 *
 * NOTE: Message is stored and copied *by value* inside the Looper, so do NOT
 * subclass it to carry extra fields — they would be sliced off and reading
 * them back is undefined behaviour. To attach a payload, put it in `data`,
 * which is a shared_ptr and therefore copies safely:
 *
 *     Message m(1);
 *     m.data = std::make_shared<std::string>("hello");
 *     ...
 *     auto* s = static_cast<std::string*>(msg.data.get());
 */
struct Message {
    Message() : what(0) {}
    Message(int w) : what(w) {}

    /* The message type. (interpretation is left up to the handler) */
    int what;

    /* Optional, type-erased payload. Copied by value (shared_ptr) so it is
     * never sliced. Cast back with static_cast on .get() in the handler. */
    std::shared_ptr<void> data;
};

/**
 * Interface for a Looper message handler.
 *
 * The Looper holds a strong reference (shared_ptr) to the message handler
 * whenever it has a message to deliver to it.  Make sure to call
 * Looper::removeMessages to remove any pending messages destined for the
 * handler so that the handler can be destroyed.
 */
class MessageHandler {
public:
    virtual ~MessageHandler();

    /**
     * Handles a message.
     */
    virtual void handleMessage(const Message& message) = 0;
};

/**
 * A simple proxy that holds a weak reference to a message handler.
 */
class WeakMessageHandler : public MessageHandler {
public:
    explicit WeakMessageHandler(const std::weak_ptr<MessageHandler>& handler);
    ~WeakMessageHandler() override;
    void handleMessage(const Message& message) override;

private:
    std::weak_ptr<MessageHandler> mHandler;
};

/**
 * A looper callback.
 */
class LooperCallback {
public:
    virtual ~LooperCallback();

    /**
     * Handles a poll event for the given file descriptor.
     *
     * Implementations should return 1 to continue receiving callbacks, or 0
     * to have this file descriptor and callback unregistered from the looper.
     */
    virtual int handleEvent(int fd, int events, void* data) = 0;
};

/**
 * Wraps a Looper_callbackFunc function pointer.
 */
class SimpleLooperCallback : public LooperCallback {
public:
    explicit SimpleLooperCallback(Looper_callbackFunc callback);
    ~SimpleLooperCallback() override;
    int handleEvent(int fd, int events, void* data) override;

private:
    Looper_callbackFunc mCallback;
};

/**
 * A polling loop that supports monitoring file descriptor events, optionally
 * using callbacks.  The implementation uses epoll() internally.
 *
 * A looper can be associated with a thread although there is no requirement
 * that it must be.
 */
class Looper : public std::enable_shared_from_this<Looper> {
public:
    enum {
        /** The poll was awoken using wake() before the timeout expired. */
        POLL_WAKE = -1,
        /** One or more callbacks were executed. */
        POLL_CALLBACK = -2,
        /** The timeout expired. */
        POLL_TIMEOUT = -3,
        /** An error occurred. */
        POLL_ERROR = -4,
    };

    /**
     * Flags for file descriptor events that a looper can monitor.
     * These flag bits can be combined to monitor multiple events at once.
     */
    enum {
        /** The file descriptor is available for read operations. */
        EVENT_INPUT = 1 << 0,
        /** The file descriptor is available for write operations. */
        EVENT_OUTPUT = 1 << 1,
        /** The file descriptor has encountered an error condition. */
        EVENT_ERROR = 1 << 2,
        /** The file descriptor was hung up. */
        EVENT_HANGUP = 1 << 3,
        /** The file descriptor is invalid. */
        EVENT_INVALID = 1 << 4,
    };

    enum {
        /**
         * Option for Looper::prepare: this looper will accept calls to addFd()
         * that do not have a callback (that is provide nullptr for the
         * callback).
         */
        PREPARE_ALLOW_NON_CALLBACKS = 1 << 0
    };

    /**
     * Creates a looper.
     *
     * If allowNonCallbacks is true, the looper will allow file descriptors to
     * be registered without associated callbacks.
     */
    explicit Looper(bool allowNonCallbacks);
    ~Looper();

    Looper(const Looper&) = delete;
    Looper& operator=(const Looper&) = delete;

    /**
     * Returns whether this looper instance allows the registration of file
     * descriptors using identifiers instead of callbacks.
     */
    bool getAllowNonCallbacks() const;

    /**
     * Waits for events to be available, with optional timeout in milliseconds.
     * Invokes callbacks for all file descriptors on which an event occurred.
     *
     * See the original libutils documentation for the full contract; behaviour
     * is unchanged.
     */
    int pollOnce(int timeoutMillis, int* outFd, int* outEvents, void** outData);
    inline int pollOnce(int timeoutMillis) {
        return pollOnce(timeoutMillis, nullptr, nullptr, nullptr);
    }

    /**
     * Like pollOnce(), but performs all pending callbacks until all data has
     * been consumed or a file descriptor is available with no callback.
     * This function will never return POLL_CALLBACK.
     */
    int pollAll(int timeoutMillis, int* outFd, int* outEvents, void** outData);
    inline int pollAll(int timeoutMillis) {
        return pollAll(timeoutMillis, nullptr, nullptr, nullptr);
    }

    /**
     * Wakes the poll asynchronously.  Can be called on any thread.
     */
    void wake();

    /**
     * Adds a new file descriptor to be polled by the looper.
     * If the same file descriptor was previously added, it is replaced.
     *
     * Returns 1 if the file descriptor was added, 0/-1 if the arguments were
     * invalid.
     */
    int addFd(int fd, int ident, int events, Looper_callbackFunc callback, void* data);
    int addFd(int fd, int ident, int events, const std::shared_ptr<LooperCallback>& callback,
              void* data);

    /**
     * May be useful for testing: read back the state registered for an fd.
     */
    bool getFdStateDebug(int fd, int* ident, int* events,
                         std::shared_ptr<LooperCallback>* cb, void** data);

    /**
     * Removes a previously added file descriptor from the looper.
     *
     * Returns 1 if the file descriptor was removed, 0 if none was previously
     * registered.
     */
    int removeFd(int fd);

    /**
     * Tell the kernel to re-check for the same events we're already checking
     * for with this fd.  Returns 1 if successfully repolled, 0 if not.
     */
    int repoll(int fd);

    /**
     * Enqueues a message to be processed by the specified handler.
     * The handler must not be null.  Can be called on any thread.
     */
    void sendMessage(const std::shared_ptr<MessageHandler>& handler, const Message& message);

    /**
     * Enqueues a message to be processed by the specified handler after the
     * specified delay (in monotonic nanoseconds).
     */
    void sendMessageDelayed(nsecs_t uptimeDelay, const std::shared_ptr<MessageHandler>& handler,
                            const Message& message);

    /**
     * Enqueues a message to be processed by the specified handler at the
     * specified time (in monotonic nanoseconds).
     */
    void sendMessageAtTime(nsecs_t uptime, const std::shared_ptr<MessageHandler>& handler,
                           const Message& message);

    /**
     * Removes all messages for the specified handler from the queue.
     */
    void removeMessages(const std::shared_ptr<MessageHandler>& handler);

    /**
     * Removes all messages of a particular type for the specified handler.
     */
    void removeMessages(const std::shared_ptr<MessageHandler>& handler, int what);

    /**
     * Returns whether this looper's thread is currently polling for more work.
     */
    bool isPolling() const;

    /**
     * Prepares a looper associated with the calling thread, and returns it.
     * If the thread already has a looper, it is returned.
     */
    static std::shared_ptr<Looper> prepare(int opts);

    /**
     * Sets the given looper to be associated with the calling thread.
     * If "looper" is null, removes the currently associated looper.
     */
    static void setForThread(const std::shared_ptr<Looper>& looper);

    /**
     * Returns the looper associated with the calling thread, or null.
     */
    static std::shared_ptr<Looper> getForThread();

private:
    using SequenceNumber = uint64_t;

    struct Request {
        int fd;
        int ident;
        int events;
        std::shared_ptr<LooperCallback> callback;
        void* data;

        uint32_t getEpollEvents() const;
    };

    struct Response {
        SequenceNumber seq;
        int events;
        Request request;
    };

    struct MessageEnvelope {
        MessageEnvelope() : uptime(0) {}
        MessageEnvelope(nsecs_t u, std::shared_ptr<MessageHandler> h, const Message& m)
            : uptime(u), handler(std::move(h)), message(m) {}

        nsecs_t uptime;
        std::shared_ptr<MessageHandler> handler;
        Message message;
    };

    const bool mAllowNonCallbacks;  // immutable

    unique_fd mWakeEventFd;  // immutable
    std::mutex mLock;

    std::vector<MessageEnvelope> mMessageEnvelopes;  // guarded by mLock
    bool mSendingMessage;                            // guarded by mLock

    // Whether we are currently waiting for work.  Not protected by a lock,
    // any use of it is racy anyway.
    volatile bool mPolling;

    unique_fd mEpollFd;          // guarded by mLock but only modified on the looper thread
    bool mEpollRebuildRequired;  // guarded by mLock

    // Locked maps of fds and sequence numbers monitoring requests.
    std::unordered_map<SequenceNumber, Request> mRequests;               // guarded by mLock
    std::unordered_map<int /*fd*/, SequenceNumber> mSequenceNumberByFd;  // guarded by mLock

    // The sequence number to use for the next fd that is added to the looper.
    SequenceNumber mNextRequestSeq;  // guarded by mLock

    // Used privately by pollOnce; runs on a single thread so needs no lock.
    std::vector<Response> mResponses;
    size_t mResponseIndex;
    nsecs_t mNextMessageUptime;  // set to LLONG_MAX when none

    int pollInner(int timeoutMillis);
    int removeSequenceNumberLocked(SequenceNumber seq);  // requires mLock
    void awoken();
    void rebuildEpollLocked();
    void scheduleEpollRebuildLocked();
};

}  // namespace worker

#endif  // WORKER_LOOPER_H
