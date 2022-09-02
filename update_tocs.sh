#! /usr/bin/env bash

# This script will update all existing tables of contents in all markdown files in this folder.

# To add a TOC to a new file, add the following two lines where you want the TOC to appear, then run this script.
# <!-- START doctoc -->
# <!-- END doctoc -->

set -e

if ! command -v doctoc &> /dev/null
then
  read -p "doctoc not found. Would you like to install it (y/N)?" install_response
  if [ $REPLY =~ ^[Yy]$ ]; then
    echo "Installing doctoc..."
    sudo apt install npm
    sudo npm install -g doctoc
    echo "Installation complete!"
  else
    echo "This script requires doctoc. Exitting."
    exit
  fi
fi

doctoc --update-only --title "## Contents" .
