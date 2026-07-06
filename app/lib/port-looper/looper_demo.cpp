// Usage example for the standalone worker::Looper with a string payload.
//
// Correct way to attach data to a Message: via Message::data (a shared_ptr),
// NOT by subclassing Message — a subclass gets sliced when the Looper copies
// it by value, and reading the sliced-off fields back crashes (core dump).

#include "looper/Looper.h"

#include <unistd.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

using namespace worker;
using namespace std;

// 1 millisecond expressed in nanoseconds (the unit sendMessageDelayed wants).
constexpr nsecs_t ms = 1000000LL;

// Current wall-clock time as "HH:MM:SS.mmm" for log lines.
static string nowStr() {
    auto now = chrono::system_clock::now();
    time_t t = chrono::system_clock::to_time_t(now);
    auto msPart = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    tm tmv{};
    localtime_r(&t, &tmv);
    char buf[16];
    strftime(buf, sizeof(buf), "%H:%M:%S", &tmv);
    ostringstream os;
    os << buf << '.' << setfill('0') << setw(3) << msPart.count();
    return os.str();
}

class AppHander {
private:
    shared_ptr<MessageHandler> pMessageHandler = nullptr;
    shared_ptr<Looper> pLooper = nullptr;

public:
    static AppHander& getInstance() {
        static AppHander app;
        return app;
    }

    void onHandleMessage(const Message& message) {
        // Cast the type-erased payload back to its real type.
        
        auto* str = static_cast<string*>(message.data.get());
        std::cout << "[" << nowStr() << "] Received (what=" << message.what << "): "
                  << (str ? *str : "<null>") << std::endl;
        AppHander::getInstance().sendMessageToApp("hello message (after 1.5s)", 1500);
    }

    void setMessageHander(shared_ptr<MessageHandler> pMessageHandler2) {
        pMessageHandler = pMessageHandler2;
    }

    void setLooper(shared_ptr<Looper> pLooper2) { pLooper = pLooper2; }

    // delayMs in milliseconds (converted to nanoseconds for the Looper).
    void sendMessageToApp(const string& str, long delayMs) {
        if (pLooper != nullptr) {
            Message message(1);
            message.data = make_shared<string>(str);  // payload survives the by-value copy
            pLooper->sendMessageDelayed(delayMs* 1000000 , pMessageHandler, message);
        }
    }
};

class MessageProcess : public MessageHandler {
public:
    void handleMessage(const Message& message) override {
        AppHander::getInstance().onHandleMessage(message);
    }
};

int main() {
    auto looper = std::make_shared<Looper>(/*allowNonCallbacks=*/false);
    AppHander::getInstance().setMessageHander(make_shared<MessageProcess>());
    AppHander::getInstance().setLooper(looper);

    // Looper thread: just pump pollOnce.
    thread t([&]() {
        while (1) {
            looper->pollOnce(-1);
        }
    });

    // Producer thread: one message after 1.5s, then a steady stream every 500ms.
    thread t2([&]() {
        std::cout <<  nowStr() << "\n" << endl;
        AppHander::getInstance().sendMessageToApp("hello first message (after 1.5s)", 1500);
    });

    t.join();
    t2.join();
    return 0;
}
