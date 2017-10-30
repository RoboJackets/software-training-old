#include "Board.h"
#include <algorithm>
#include <iostream>

/**
 * constructor
 *  - Fill data array with Empty Markers
 *  - Set winner to Empty
 */


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


/**
 * getMarker
 * @param r unsigned integer, row coordinate of place to read
 * @param c unsigned integer, column coordinate of place to read
 * @return marker contained at <r,c>
 */


/**
 * isOver
 * @return true if the game is over, false otherwise
 * @note The game is over if no more moves can be made or if one player has won the game
 */


/**
 * getWinner
 * @return The current winner, or Empty if tie or in-progress game
 */


/**
 * locationToIndex
 * @param r unsigned integer, row coordinate
 * @param c unsigned integer, column coordinate
 * @return index (of type size_t) for data array corresponding to <r,c>
 * @note data should be row-ordered, so each row of the board should be
 *       saved contiguously in data immediately following the row above it.
 */


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

