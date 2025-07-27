# submodules/ariadna_constants/./generate-files.sh

# cmake -B build -G "Unix Makefiles" -DCOPY_COMPILE_COMMANDS=ON
# cmake --build build


source .venv/bin/activate

src/can_constants/./generate-files.sh
./after_ioc.sh


if [ "$1" == "r" ]; then
  cmake -B build -G "Ninja" -DCOPY_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
else 
  cmake -B build -G "Ninja" -DCOPY_COMPILE_COMMANDS=ON 
fi

cmake --build build