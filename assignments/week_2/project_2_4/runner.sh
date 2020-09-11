#!/usr/bin/env bash

# TODO
# This script currently only checks that the light outputs are legal. It doesn't check if the outputs actually
# reflect the sensor readings in the input file.

input_file_path=$1

if [ ! -f "$input_file_path" ]; then
  echo "$input_file_path does not exist!"
  exit 1
fi

echo "Test case: $input_file_path"

program_output=$(cat $input_file_path | ./light_controller)

time=0
red_light_time=0
yellow_light_time=0
green_light_time=0

previous_line=""

has_failure=false

while IFS= read -r line; do
  echo "$time: $line"
  ((time += 1))

  if ! [[ $line =~ ^[0-1]{6}$ ]]; then
    echo "ERROR: Invalid output! Must be 6 binary digits!"
    has_failure=true
    continue
  fi

  case $line in
  "100100")
    ((red_light_time+=1))
    ;;
  "010100")
    ((yellow_light_time+=1))
    ;;
  "001100")
    ((green_light_time+=1))
    ;;
  "100010")
    ((yellow_light_time+=1))
    ;;
  "100001")
    ((green_light_time+=1))
    ;;
  *)
    echo "ERROR: Incorrect combination of lights! Controller is producing confusing or dangerous outputs!"
    has_failure=true
    ;;
  esac

  if [ "$line" != "001100" ] && [ "$line" != "100001" ]; then
    if (( green_light_time > 0 && green_light_time < 5 )); then
      echo "ERROR: The light did not stay green long enough!"
      has_failure=true
    fi
    green_light_time=0
  fi

  if [ "$line" != "010100" ] && [ "$line" != "100010" ]; then
    if (( yellow_light_time > 0 && yellow_light_time < 2 )); then
      echo "ERROR: The light did not stay yellow long enough!"
      has_failure=true
    fi
    yellow_light_time=0
  fi

  if [ "$line" != "100100" ]; then
    # Red light is only on for 1 round, so it can't be too short. That would just be an illegal transition.
    red_light_time=0
  fi

  if [ $red_light_time -gt 1 ]; then
    echo "ERROR: The intersection has been stopped for too long!"
    has_failure=true
  fi

  if [ $yellow_light_time -gt 2 ]; then
    echo "ERROR: The light has been yellow for too long!"
    has_failure=true
  fi

  if [ $green_light_time -gt 5 ]; then
    echo "ERROR: The light has been green for too long!"
    has_failure=true
  fi

  if [ ! -z $previous_line ] && [ $line != $previous_line ]; then
    if [ "$line" == "010100" ] && [ $previous_line != "001100" ]; then
      echo "ERROR: Illegal transition to yellow light."
      has_failure=true
    fi
    if [ "$line" == "001100" ] && [ $previous_line != "100100" ]; then
      echo "ERROR: Illegal transition to green light."
      has_failure=true
    fi
    if [ "$line" == "100010" ] && [ $previous_line != "100001" ]; then
      echo "ERROR: Illegal transition to yellow light."
      has_failure=true
    fi
    if [ "$line" == "100001" ] && [ $previous_line != "100100" ]; then
      echo "ERROR: Illegal transition to green light."
      has_failure=true
    fi
    if [ "$line" == "100100" ] && [ $previous_line != "010100" ] && [ $previous_line != "100010" ]; then
      echo "ERROR: Illegal transition to red light."
      has_failure=true
    fi
  fi

  previous_line=$line

done <<< "$program_output"

if [ "$has_failure" == "false" ]; then
  echo "SUCCESS!"
else
  echo "ERRORS OCCURRED"
fi
