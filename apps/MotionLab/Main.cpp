#include "apps/MotionLab/MotionLabApp.h"

#include <string_view>

int main(int argc, char* argv[])
{
    motion_lab::MotionLabApp app { };
    if (!app.initialize()) {
        return 1;
    }

    for (int i { 1 }; i < argc; ++i) {
        const std::string_view arg { argv[i] };
        if (arg == "--slalom" || arg == "-s" || arg == "slalom") {
            app.setSlalom(true);
        } else if (arg == "--circuit" || arg == "-c" || arg == "circuit") {
            app.toggleCircuit();
        }
    }

    app.run();
    return 0;
}
