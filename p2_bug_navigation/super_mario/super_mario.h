#ifndef BUG_NAVIGATION_SUPER_MARIO_H
#define BUG_NAVIGATION_SUPER_MARIO_H

#include <cstdlib>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

#define PRINT_WINDOW  20

/**
 * Converts Mario state number to the corresponding character value.
 * @param  state The Mario state number.
 * @return       The character associated with the state number.
 */
char marioStateToString(int state);

/**
 * Prints a simple visualization of the game's state.
 * @param  events The events sequence used in the game.
 * @param   state The current Mario state number. 
 * @param     idx The current index in the events sequence.
 */
void printState(std::vector<char>& events, int state, int idx);

/**
 * Generates a random events sequence of specified length.
 * @param  numEvents The number of events in the sequence.
 * @return           The generated sequence. 
 */
std::vector<char> randomEventSequence(int numEvents);

/**
 * Loads a sequence of events from a file.
 * @param  filePath The relative path of the file to load from. 
 * @return          The sequence of events to load from, empty if filePath is invalid.
 */
std::vector<char> readCourseData(const std::string &filePath);

/**
 * Runs a Super Mario game given a sequence of events, optionally displaying in the terminal. 
 * @param   events The events to run the game on. 
 * @param  display Whether to display the game on stdout, defaults to true.
 * @return         The score of the game. 
 */
int playGame(std::vector<char> &events, bool display = true);

#endif  // BUG_NAVIGATION_SUPER_MARIO_H