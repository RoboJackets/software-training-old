# Project 2.3
Today we will be looking at an already written class and filling in parts of the code.
Often when we are working with larger code bases you have to get familiar with the
different parts of the code.

Often pulling up someone else's code is a daunting task but hopefully we can
give you some advice on to how to approach that. This exercise is rather difficult.

We are looking at a commandline implementation of a tic tac toe game that you can play.

## Main function
The first thing you will want to do is find out where the main function is. Since
this is the first method run it can usually give you some clue to the structure
of the code and you can start tracing from there. Now for this example it is
pretty easy to find, just look at the bottom on main.cpp.

So it looks there are a couple helper methods in this file. Just based off of the
method names we can begin to guess what these methods might do.

```c++
void generateComputerMove(const Board &board, unsigned int &r_out, unsigned int &c_out) {
  static random_device rand_dev;
  static default_random_engine engine(rand_dev());
  static uniform_int_distribution<unsigned int> uniform_dist(0,2);
  do {
    r_out = uniform_dist(engine);
    c_out = uniform_dist(engine);
  } while(board.getMarker(r_out, c_out) != Marker::Empty);
}
```

From this method we can guess at a couple things. First the computer moves are
being generated of the fly. The computer move is based off of some Board objects.
There is a lot of reference to random numbers, so we can assume that they are guessing
at locations randomly and then checking if that location is empty.

```c++
void getMoveFromPlayer(unsigned int &r_out, unsigned int &c_out) {
  static regex coordinate_pattern("[0-2],[0-2]");
  do {
    cout << "Your move (r,c): ";
    string input;
    cin >> input;
    if(regex_match(input,coordinate_pattern)) {
      auto comma_loc = input.find(',');
      r_out = static_cast<unsigned int>(stoi(input.substr(0,comma_loc)));
      c_out = static_cast<unsigned int>(stoi(input.substr(comma_loc+1)));
      return;
    } else {
      cout << "Sorry, I couldn't understand your coordinates!" << endl;
    }
  } while(true);
}
```

From this method it looks like it is taking in two locations, likely board locations,
and then making sure they are valid before making that move.

```c++
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
```

This method looks like it is just return what character we should use for the
different markers that could be at that location on the board.

```c++
void printBoard(const Board &board) {
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
```

Here we are taking in a board object and printing it. We can see that the
charFromMarker method is called.

Now let us look at the actual main method

```c++
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
```

So it looks like this loops does the following
1. prints out the board
2. gets a move from the player until it is valid
3. places the players marker
4. prints the board
5. generates a move for the computer
6. places the computers marker

Finally the main decides the winner

```
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
```

from this code we can see that it is using the winner marker to determine which
player has won. That mean the markers are unique to each player.

# Board class
Now that we have looked at the main file and have a general idea of what is happening
we should look at the Board object itself. A good place to start is the
header file

## Looking at the header file
Hopefully the person who wrote the code properly documented it and left some
useful comments in the header file. Let's open up the Board.h file.

```c++
enum class Marker {
  Empty,
  X,
  O
};
```

This is a definition of an enum, think of it like a special variable type that can only hold
one of the listed options as a value.

```c++
class Board {
```

So we have the definition of the Board class itself.

```c++
class Board {
public:

  Board();

  bool placeMarker(int r, int c, Marker marker);

  Marker getMarker(int r, int c) const;

  bool isOver() const;

  Marker getWinner() const;

private:

  std::array<Marker,9> data;

  Marker winner;

  size_t locationToIndex(unsigned int r, unsigned int c) const;

  void updateWinner(unsigned int last_r, unsigned int last_c);

};
```

So what can we tell from the different parts of this code?

We can tell that a board has an array of markers with size 9. This matches exactly
with a 3x3 board.

```c++
  std::array<Marker,9> data;
```

## Looking at Board.cpp

Unfortunately it looks like this class is missing some key functionality. There
are a lot of TODO's. The rest of this exercise will be filling in those missing
methods.

Starting off with the constructor

```c++
/**
 * constructor
 *  - Fill data array with Empty Markers
 *  - Set winner to Empty
 */
Board::Board() {
  // TODO
}
```

Now our class has a member variable winner. This is the marker that was last used
so we know who the winner is (the last person to move into a winning condition is the winner).
The comments tell us we need to fill our board (array) with empty markers, and
we need to set the winner to empty.

Let's test what we have by running the code

```bash
    mkdir build
    cd build
    cmake ..
    make
    ./exercise_2_2
```

<details>
  <summary>Answer</summary>

  ```c++
    std::fill(data.begin(), data.end(), Marker::Empty);
    winner = Marker::Empty;
  ```
</details>

Next we have the method to get a marker

```c++
/**
 * getMarker
 * @param r  integer, row coordinate of place to read
 * @param c  integer, column coordinate of place to read
 * @return marker contained at <r,c>
 */
Marker Board::getMarker(int r, int c) {
  // TODO using the helper return the marker at that location data[index]
  // TODO use helper method locationToIndex
}
```

So this method should get the marker value at the correct location in the
std::array<Marker> data. There is a helper method to convert a row and col into
a index in that array. This should method should just be accessing the array
at that point. Now let's look at that helper method.

```c++
/**
 * locationToIndex
 * @param r  integer, row coordinate
 * @param c  integer, column coordinate
 * @return index (of type size_t) for data array corresponding to <r,c>
 * @note data should be row-ordered, so each row of the board should be
 *       saved contiguously in data immediately following the row above it.
 */
size_t Board::locationToIndex(int r, int c) {
  // TODO what is the formula for the index into data based off of row and col?
  // indexes of the board are
  // data[0], data[1], data[2]
  // data[3], data[4], data[5]
  // data[6], data[7], data[8]
  return 0;
}

```


<details>
  <summary>Answer getMarker</summary>

  ```c++
    return data[locationToIndex(r,c)];
  ```
</details>


<details>
  <summary>Answer locationToIndex</summary>

  ```c++
    return ( r * 3 ) + c;
  ```
</details>

Now we can see that we need to find a formula for how to convert a row and col
into the correct location in a flattened 1d array.

```c++
/**
 * isOver
 * @return true if the game is over, false otherwise
 * @note The game is over if no more moves can be made or if one player has won the game
 */
bool Board::isOver() {
  // TODO
}
```

Finally we need to check if the game has ended. There are a couple conditions
listed in the comments. See if you can figure out a good way to test your methods.


<details>
  <summary>Answer</summary>

  ```c++
      if(std::count(data.begin(), data.end(), Marker::Empty) == 0) {
        return true;
      } else if(getWinner() != Marker::Empty) {
        return true;
      } else {
        return false;
      }
  ```
</details>

