#include "window.hpp"

int main()
{
    // OpenGL window
    Window window(800, 600, "Hello World");
    window.setBackgroundColor(0.2f, 0.3f, 0.3f, 1.0f);

    // render loop
    while (!window.shouldClose())
    {
        // wipe out
        window.wipeOut();

        // display
        window.display();
        
        // user inputs
        window.processUserInputs();
    }

    return 0;
}
