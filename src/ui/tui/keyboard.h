#pragma once

namespace ntn::ui {

class Keyboard {
public:
    // Returns true if a key has been pressed
    static bool kbhit();
    
    // Reads a single character without waiting for Enter
    static char getch();
};

} // namespace ntn::ui
