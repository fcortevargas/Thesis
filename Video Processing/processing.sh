#!/bin/bash

# Define the source directory
source_dir="../Videos/Unprocessed"

# Define the destination directory
destination_dir="../Videos/Processed"

# Loop through each folder in the source directory
for folder in "$source_dir"/*/; do
    # Extract the parent folder name
    parent_folder=$(basename "$folder")
    # Extract the prefix from the parent folder name
    prefix="${parent_folder%%_*}"
    # Loop through "Top" and "Side" folders inside the current folder
    for subfolder in "$folder"*/; do
        # Extract the subfolder name ("Top" or "Side")
        subfolder_name=$(basename "$subfolder")
        # Loop through the "Individuals" folders inside the "Top" and "Side" folders
        for individuals_folder in "$subfolder"Individuals/; do
            # Loop through each .mp4 file in the "Individuals" folder
            for mp4_file in "$individuals_folder"*.mp4; do
                if [ -f "$mp4_file" ]; then
                    # Extract the file name without extension
                    file_name=$(basename "$mp4_file" .mp4)
                    # Extract the suffix from the file name
                    suffix="${file_name##*_}"
                    # Calculate the new index based on prefix and suffix
                    new_index=$((prefix + suffix - 1))
                    # Rename the file based on the new index and subfolder name
                    new_file_name=$(printf "%s/%d_%s.mp4" "$destination_dir/$subfolder_name" "$new_index" "$subfolder_name")
                    # Move the file to the destination directory
                    mv "$mp4_file" "$new_file_name"
                    echo "Moved: $mp4_file to $new_file_name"
                fi
            done
        done
    done
done

echo "Script execution completed."
