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
./osm_release -bo stuttgart.bin stuttgart-regbez-251010.osm.pbf
```

The output should look something like this:
```
Parsing stuttgart-regbez-251010.osm.pbf...                                 
Parsing done [6.54s]                                                       
Number of buildings: 1683616                                                                                                                          
Number of streets: 195645                                                                                                                             
Number of admin areas: 1465                                                                                                                           
Preprocessing...                                                           
        Building hierarchical admin areas kdtrees...                                                                                                  
        Hierarchical admin areas kdtrees built [300µs]                                                                                                
        Point in polygon test for buildings using hierarchical admin areas...                                                                         
        Assigned areas to buildings [3.64s]                                                                                                           
        Building buildings kdtree...                                                                                                                  
        KDtree built [513.40ms]                                            
        Building streets kdtree...                                                                                                                    
        KDtree built [411.48ms]                                                                                                                       
        Interpolating street names for buildings without a street assigned to them...
        Street names assigned [10.30s]                                                                                                                                                                                                                                   
Preprocessing done [14.87s]    
Serializing to stuttgart.bin...                                            
Serialization done [2.05s]                                                 
Server started at http://localhost:8080
```

Now you can open `http://localhost:8080` in any browser and explore the hosted map.

## Reverse Geocoder

The hosted map should look like this:

![Leaflet Map](imgs/map.png)

In the top left corner the user can select what objects should be displayed:
- **Building**: Shows all buildings in the current viewport. If the whole address is known, the marker will be blue, if the street was interpolated it will be green.
- **Streets**: Shows all streets in the current viewport. Streets are orange lines.
- **Admin areas**: Shows all admin areas in the current viewport. Admin areas are red opaque polygons.
- **Dynamic** *(Recommended)*: Viewable objects are dynamically displayed depending on the zoom level. Other options cannot be selected if this is checked.

Clicking on the map will select the nearest (visible) object and display the respective name.
If the nearest object is a building, the whole address will be displayed.

**Example**: Buildings and Streets checkboxes are checked. User clicks near a street:
![Street selected](imgs/street.png)

and later on top of a building:
![Building selected](imgs/building.png)

Each object will be marked with the color red and a popup box will display the name.

**Example**: Dynamic object display is selected and user zoomed out a lot. Now clicks on an admin area:
![Admin area selected](imgs/area.png)


# Useful links
- [geofabrik](https://download.geofabrik.de/index.html)