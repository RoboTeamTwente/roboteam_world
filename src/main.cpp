#include "Handler.h"

int main(int argc, char** argv) {
    auto itMarple = std::find(argv, argv + argc, std::string("-marple"));
    bool logInMarpleFormat = itMarple != argv + argc;

    Handler handler(logInMarpleFormat);

    handler.start();
    return 0;
}