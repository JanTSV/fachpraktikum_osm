# Requirements

- g++
- make
- libosmium / protozero
- boost

# How to

## Build the project

**Note**: No data sets are provided here, please download them yourself using the [links](#useful-links) below.

First create a dir `build/` in the project root dir:
```bash
mkdir build
```

The project is build with a `Makefile`. To build the release run
```bash
make release
```

to build the debug version run:
```bash
make debug
```

This should create `osm_release` / `osm_debug` executables.

## Use command-line arguments
You can always run
```bash
# <EXECUTABLE> : osm_release | osm_debug
./<EXECUTABLE> --help
```

to get the following print:
```
SYNOPSIS
        ./osm_release [-h] [-bi] [-bo <output file>] [-p <port>] <input file>

OPTIONS
        -h, --help  Print this help
        -bi, --binary_in
                    Input file is in binary format

        -bo, --binary_out <output file>
                    Optional path to output binary file

        -p, --port <port>
                    Port number for the localhost. DEFAULT = 8080

        <input file>
                    Path to OSM file
```

Please acknowledge the order of the arguments in the `SYNOPSIS` block.

**Example**: You want to read in raw OSM data and serialize it to another file (`stuttgart.bin`)
```bash
./osm_release -bo stuttgart.bin stuttgart-regbez-251010.osm.pbf
```

**Example**: Data has already been serialized in a former step. Now the bin data should be parsed and the map should be hosted on port `8090` instead of the default `8080`
```bash
./osm_release -p 8090 -bi stuttgart.bin
```

## Parse and display OSM data

Lets take the same example from above:
```bash
./osm_release -p 8090 -bi stuttgart.bin
```

The output should look something like this:
```
Loading binary data from stuttgart_naive.bin...
Loaded binary data [907.84ms]
Number of buildings: 3365587
Number of streets: 195642
Number of admin areas: 1474
Server started at http://localhost:8090
```

Now you can open `http://localhost:8090` in any browser and explore the hosted map. Each building should have a marker node on it.

# Useful links
- [geofabrik](https://download.geofabrik.de/index.html)