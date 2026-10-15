#include "apps/MotionLab/MotionLabApp.h"

int main(int argc, char* argv[])
{
    static_cast<void>(argc);
    static_cast<void>(argv);

    motion_lab::MotionLabApp app { };
    if (!app.initialize()) {
        return 1;
    }

    app.run();
    return 0;
}
