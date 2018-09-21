#ifndef WEEK6_BOARD_H
#define WEEK6_BOARD_H

#include <array>

enum class Marker {
    Empty,
    X,
    O
};

/* Task: Create and implement a class, Board, which represents a TicTacToe playing board
 *
 * See Board.cpp for detailed descriptions of all method members
 *
 * Public members:
 * constructor
 * placeMarker(r,c,marker)
 * getMarker(r,c)
 * isOver()
 * getWinner()
 *
 * Private members:
 * data - an array of 9 Markers
 * winner - a single marker representing the current winner
 * locationToIndex(r,c)
 * updateWinner(last_r, last_c)
 *
 */



#endif //WEEK6_BOARD_H
