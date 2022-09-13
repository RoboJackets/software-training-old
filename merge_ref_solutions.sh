#! /usr/bin/env bash

# merge_ref_solutions.sh
#
# This is a helper script meant for trainers and project developers. It is designed to help you
# incorporate code from the reference solutions into your current project solutions. Beyond just
# a blind copy, it will prompt you to merge files so you can keep the parts you've written that
# you don't want to change.
#
# Node: This script uses kdiff3 for merging. You'll need to install it.
#
# Options:
# --rm-origs             : Removes the *.orig files created by kdiff3 after merging
# --package PACKAGE_NAME : Only merge the given package
# --overwrite-all        : Bypasses all merge prompts and copies all files. This will erase any
#                          changes you've made that aren't in the reference solutions.


RM_ORIG_FILES=0
PACKAGE_NAME=""
OVERWRITE_ALL=0

options=$(getopt \
  --longoptions "rm-origs,package:,overwrite-all" \
  --name "$(basename "$0")" \
  --options "" \
  -- "$@"
)
eval set --$options
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rm-origs)
      RM_ORIG_FILES=1
      shift 1
      ;;

    --package)
      PACKAGE_NAME=$2
      shift 2
      ;;

    --overwrite-all)
      OVERWRITE_ALL=1
      shift 1
      ;;

    --)
      break
      ;;

    *)
      echo "Unknown option: $1"
      break
      ;;
  esac
done

if ! command -v kdiff3 &> /dev/null
then
    echo "Could not find kdiff3, which is required. Please install it."
    echo "eg. via apt: sudo apt install kdiff3"
    exit 1
fi

for solution_file in $(find reference_solutions/$PACKAGE_NAME -type f -print)
do
    current_file=$(echo $solution_file | sed 's/reference_solutions\///')
    if [ "$current_file" = "COLCON_IGNORE" ]
    then
        continue
    fi

    if [ ! -f $current_file ]
    then
        echo "Copying new file: $current_file"
        cp $solution_file $current_file
        continue
    fi

    if cmp --silent $current_file $solution_file
    then
        echo "No changes in $current_file"
        continue
    fi

    if [ "$OVERWRITE_ALL" ]
    then
        echo "Overwriting $current_file"
        cp $solution_file $current_file

    else
      kdiff3 --merge --L1 'Current Solution' --L2 'Reference Solution' --output "$current_file" "$current_file" "$solution_file"

      if [ "$RM_ORIG_FILES" ]
      then
          rm $current_file.orig
      fi

    fi

done

exit 0
