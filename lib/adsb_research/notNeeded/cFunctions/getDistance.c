#include <math.h>

int getDistance(double n0, double e0, double d0, double n1, double e1, double d1){
    return round(sqrt((n0 - n1)*(n0 - n1) + (e0 - e1)*(e0 - e1) + (d0 - d1)*(d0 - d1)));
}