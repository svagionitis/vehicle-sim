#include "GameApp.h"

int main(int argc, char* argv[])
{
    static_cast<void>(argc);
    static_cast<void>(argv);

    earth_patrol::GameApp app { };
    if (!app.initialize()) {
        return 1;
    }

    app.run();
    return 0;
}
