submodules/ariadna_constants/./generate-files.sh

cmake -B build -G "Ninja" -DCOPY_COMPILE_COMMANDS=ON
cmake --build build