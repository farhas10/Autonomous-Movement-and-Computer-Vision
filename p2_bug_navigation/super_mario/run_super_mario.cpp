#include <super_mario.h>

int main(int argc, char *argv[]) {
    // This sets up the random number generator.
    srand((unsigned) time(NULL));

    int NUM_EVENTS = 50;     // Number of events in the random sequence.

    std::vector<char> events;  // A vector to store the events.
    if (argc >= 2) {
        // If the path to a file was provided, load that environment. This is
        // useful for testing on the test environments in the folder "test".
        std::string filePath = argv[1];
        events = readCourseData(filePath);
    }
    else {
        // If no pre-determined environment was found, load a random one.
        events = randomEventSequence(NUM_EVENTS);
    }

    int score = playGame(events);
}