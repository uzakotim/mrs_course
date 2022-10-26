# TODO

* [X] remove hint/code leftover in `formation.cpp`: `// form a vertical column` comment
* [X] remove hint/code leftover in `formation.h`: state machine enum
* [X] emphasize that formation planning is done relative to the virtual leader
* [X] give a hint in the warning when formation paths are too long, e.g.: [Wrapper]: reshapeFormation(): paths contain points that are too far from initial leader's position (are you planning relative to leader's position?)
* [X] add world axes to default rviz
* [X] add markers to marker array so that they don't have to be added to rviz manually
* [X] tune noise of default multilateration so that it is not possible to get too many points with it
