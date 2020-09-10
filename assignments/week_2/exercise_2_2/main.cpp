#include <iostream>
#include <regex>
#include <random>
#include "Board.h"

using namespace std;

void generateComputerMove(Board &board, int &r_out, int &c_out) {




  static random_device rand_dev;
  static default_random_engine engine(rand_dev());
  static uniform_int_distribution< int> uniform_dist(0,2);
  do {
    r_out = uniform_dist(engine);
    c_out = uniform_dist(engine);
  } while(board.getMarker(r_out, c_out) != Marker::Empty);
}

void getMoveFromPlayer(int &r_out, int &c_out) {
  static regex coordinate_pattern("[0-2],[0-2]");
  do {
    cout << "Your move (r,c): ";
    string input;
    cin >> input;
    if(regex_match(input,coordinate_pattern)) {
      auto comma_loc = input.find(',');
      r_out = static_cast< int>(stoi(input.substr(0,comma_loc)));
      c_out = static_cast< int>(stoi(input.substr(comma_loc+1)));
      return;
    } else {
      cout << "Sorry, I couldn't understand your coordinates!" << endl;
    }
  } while(true);
}

char charForMarker(Marker m) {
  switch (m) {
    case Marker::Empty:
      return ' ';
    case Marker::X:
      return 'X';
    case Marker::O:
      return 'O';
  }
}

void printBoard(Board &board) {
  std::string layout;
  auto n = 3;
  for(int r = 0; r < n; r++) {
    for(int c = 0; c < n; c++) {
      layout += charForMarker(board.getMarker(r,c));
      if(c != (n-1)) {
        layout += "|";
      }
    }
    if(r != (n-1)) {
      layout += "\n-----\n";
    }
  }
  cout << layout << endl << endl;
}

int main() {
  cout << "Welcome to TicTacToe!\n\n";
  cout << "You will play O's and the computer will play X's\n";
  cout << "Good luck!\n\n";

  Board board;

  while(!board.isOver()) {
    printBoard(board);
    int r_move;
    int c_move;
    do {
      r_move = 0;
      c_move = 0;
      getMoveFromPlayer(r_move, c_move);
    } while(!board.placeMarker(r_move, c_move, Marker::O));
    printBoard(board);
    if(board.isOver()) {
      break;
    }
    cout << "Computer's move: ";
    r_move = 0;
    c_move = 0;
    generateComputerMove(board, r_move, c_move);
    cout << r_move << "," << c_move << endl;
    board.placeMarker(r_move, c_move, Marker::X);
  }

  cout << "\n\n";

  printBoard(board);

  switch(board.getWinner()) {
    case Marker::Empty:
      cout << "You tied!" << endl;
      break;
    case Marker::O:
      cout << "You won!" << endl;
      break;
    case Marker::X:
      cout << "You lost!" << endl;
      break;
  }

  return 0;
}
