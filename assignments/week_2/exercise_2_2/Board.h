#pragma once

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

class Board {
public:

  Board();

  bool placeMarker(int r, int c, Marker marker);

  Marker getMarker(int r, int c);

  bool isOver();

  Marker getWinner();

private:

  std::array<Marker,9> data;

  Marker winner;

  size_t locationToIndex( int r,  int c);

  void updateWinner(int last_r, int last_c);

};


