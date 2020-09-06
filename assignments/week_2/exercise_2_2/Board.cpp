#include "Board.h"
#include <algorithm>
#include <iostream>

/**
 * constructor
 *  - Fill data array with Empty Markers
 *  - Set winner to Empty
 */
Board::Board() {
  std::fill(data.begin(), data.end(), Marker::Empty);
  winner = Marker::Empty;
}

/**
 * placeMarker
 * @param r unsigned integer, row coordinate where marker should be placed
 * @param c unsigned integer, column coordinate where marker should be placed
 * @param marker Marker, marker to be placed
 *
 * @return true, if <r,c> was Empty and now contains marker; false otherwise
 *
 * @note Be sure to call updateWinner after placing the marker
 */
bool Board::placeMarker(unsigned int r, unsigned int c, Marker marker) {
  auto &space = data[locationToIndex(r,c)];

  if(space != Marker::Empty) {
    return false;
  }

  space = marker;

  updateWinner(r,c);

  return true;
}

/**
 * getMarker
 * @param r unsigned integer, row coordinate of place to read
 * @param c unsigned integer, column coordinate of place to read
 * @return marker contained at <r,c>
 */
Marker Board::getMarker(unsigned int r, unsigned int c) const {
  return data[locationToIndex(r,c)];
}

/**
 * isOver
 * @return true if the game is over, false otherwise
 * @note The game is over if no more moves can be made or if one player has won the game
 */
bool Board::isOver() const {
  if(std::count(data.begin(), data.end(), Marker::Empty) == 0) {
    return true;
  } else if(getWinner() != Marker::Empty) {
    return true;
  } else {
    return false;
  }
}

/**
 * getWinner
 * @return The current winner, or Empty if tie or in-progress game
 */
Marker Board::getWinner() const {
  return winner;
}

/**
 * locationToIndex
 * @param r unsigned integer, row coordinate
 * @param c unsigned integer, column coordinate
 * @return index (of type size_t) for data array corresponding to <r,c>
 * @note data should be row-ordered, so each row of the board should be
 *       saved contiguously in data immediately following the row above it.
 */
size_t Board::locationToIndex(unsigned int r, unsigned int c) const {
  return ( r * 3 ) + c;
}

/**
 * updateWinner
 * @param last_r unsigned integer, row coordinate of last move
 * @param last_c unsigned integer, column coordinate of last move
 * @note pseudo-code algorithm:
 *       col=row=diag=rdiag=0
 *       winner=Empty
 *       for i=0 to 2
 *         if cell[x,i]=player then col++
 *         if cell[i,y]=player then row++
 *         if cell[i,i]=player then diag++
 *         if cell[i,2-i]=player then rdiag++
 *       if row=n or col=n or diag=n or rdiag=n then winner=player
 */
void Board::updateWinner(unsigned int last_r, unsigned int last_c) {
  auto hor = 0;
  auto ver = 0;
  auto diag1 = 0;
  auto diag2 = 0;
  auto player = getMarker(last_r, last_c);
  for(unsigned int i = 0; i < 3; i++) {
    if (getMarker(last_r, i) == player)
      hor++;
    if (getMarker(i,last_c) == player)
      ver++;
    if (getMarker(i,i) == player)
      diag1++;
    if (getMarker(i,2-i) == player)
      diag2++;
  }

  if (hor == 3 || ver == 3 || diag1 == 3 || diag2 == 3) {
    winner = player;
  }
}
