#include <cstdlib>
#include <iostream>
#include "apps/MontsoftOffroad/GameApp.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[])
{
    try {
        montsoft_offroad::GameAppConfig config {};
        config.windowWidth = 1280;
        config.windowHeight = 800;
        config.windowTitle = "Montsoft Offroad (1988) - C++17 & SDL2";
        config.initialPalette = montsoft_offroad::PaletteMode::CgaPalette0;

        montsoft_offroad::GameApp app(config);
        if (!app.initialize()) {
            std::cerr << "[MontsoftOffroad] Initialization failed." << std::endl;
            return EXIT_FAILURE;
        }

        app.run();
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "[MontsoftOffroad] Fatal exception: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "[MontsoftOffroad] Unknown fatal error occurred." << std::endl;
        return EXIT_FAILURE;
    }
}
