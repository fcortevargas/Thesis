#!/bin/bash

# Define the directory containing the videos
directory="../Videos/Compressed/Side"

# Go to the directory
cd "$directory" || exit

# Iterate through each video file
for file in *.mp4; do
    # Extract the number from the filename
    number=$(echo "$file" | grep -oE '^[0-9]+')

    # Calculate the number of digits in the number
    num_digits=${#number}

    # Calculate the number of zeros to prepend
    num_zeros=$((3 - num_digits))

    # Prepend the zeros to the number
    padded_number="$number"
    for ((i=0; i<num_zeros; i++)); do
        padded_number="0$padded_number"
    done

    # Form the new filename
    new_filename="${padded_number}_Side.mp4"

    # Rename the file
    mv "$file" "$new_filename"
done
