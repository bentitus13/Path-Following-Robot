#include "Coordinate.h"

Coordinate::Coordinate(int xx, int yy, double thetaa) {
    x = xx;
    y = yy;
    theta = thetaa;
}

Coordinate::Coordinate() {
    x = -1;
    y = -1;
}

void Coordinate::init(int xx, int yy, double thetaa) {
    x = xx;
    y = yy;
    theta = thetaa;
}
