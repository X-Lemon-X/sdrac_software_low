#!/bin/sh

# Get the number of commits in the repository
minor_version=$(git rev-list --all --count)
major_version=$2

# Define the destination file
dst_file="$1/app/version.hpp"

# Generate the version.hpp file
echo "#ifndef VERSION_HPP" > $dst_file
echo "#define VERSION_HPP" >> $dst_file
echo "" >> $dst_file
echo "#define VERSION_MAJOR $major_version" >> $dst_file
echo "#define VERSION_MINOR $minor_version" >> $dst_file
echo "" >> $dst_file
echo "#endif // VERSION_HPP" >> $dst_file