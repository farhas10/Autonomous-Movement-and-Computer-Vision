#include <super_mario.h>

char marioStateToString(int state) {
    if (state == 0) {
        return '.';
    }
    else if (state == 1) {
        return 'm';
    }
    else if (state == 2) {
        return 'M';
    }
    else if (state == -1) {
        return 'x';
    }
    return '?';  // Default case for invalid state.
}

void printState(std::vector<char>& events, int state, int idx) {
    int start = std::max(idx - PRINT_WINDOW / 2, 0);
    int end = std::min(start + PRINT_WINDOW, (int)events.size());
    std::cout << '\r';
    for (int i = start; i < end; i++) {
        if (i == idx) std::cout << marioStateToString(state);
        else std::cout << events[i];
    }
    std::cout << std::setw(PRINT_WINDOW) << std::flush;
}

std::vector<char> randomEventSequence(int numEvents) {
    std::vector<char> events = {'_'};

    while (events.size() < numEvents) {
        char action = '_';  // Default is no action.
        // Get a random number.
        int random = rand() % 10;
        // Pick the action based on the number.
        if (random == 0) action = '-';
        else if (random == 1) action = '+';
        events.push_back(action);
    }

    return events;
}

std::vector<char> readCourseData(const std::string &filePath) {
    std::vector<char> events;
    std::ifstream fileStream(filePath, std::ios::in);

    if (!fileStream.is_open()) {
        std::cerr << "Could not open the file: " << filePath << std::endl;
        return events;
    }

    char ch;
    while (fileStream.get(ch)) {
        if (ch != '\n') {
            events.push_back(ch);
        }
    }

    fileStream.close();
    return events;
}

int playGame(std::vector<char> &events, bool display) {    
    /**
     * Mario States:
     *  0: Small mario
     *  1: Regular mario
     *  2: Big mario
     * -1: Dead mario
     **/
    int marioState = 1;
    int idx = 0;      // The index of the event sequence.
    int points = 0;   // The current number of points.
    float SLEEP_TIME = 0.3;  // seconds.

    if (display) {
        // Print the legend.
        std::cout << "Mario States:\n  Small = " << marioStateToString(0);
        std::cout << "  Regular: " << marioStateToString(1);
        std::cout << "  Big: " << marioStateToString(2);
        std::cout << "  Dead: " << marioStateToString(-1) << "\n\n";
        std::cout << "Events:\n  No event = _  Turtle: -  Mushroom: +\n\n";

        // Print the initial state.
        printState(events, marioState, idx);
        usleep(SLEEP_TIME * 1e6);
    }

    while (idx < events.size()) {
        char currentEvent = events[idx];

        // *** Task: Implement the Super Mario state machine *** //

        // *** End student code *** //

        idx++;
        if (display) {
            printState(events, marioState, idx);
            usleep(SLEEP_TIME * 1e6);
        }
        if (marioState == -1) break;
    }

    if (display) {
        std::cout << std::endl;
        std::cout << "\nTotal points: " << points << std::endl;
        std::cout << "\nFinal Mario state: " << marioStateToString(marioState) << "  ";
        std::cout << "Run length: " << idx << std::endl;
    }

    return points;
}